// ==== I2C 128x128 OLED (SH1107) + DDS PWM + 4x4 Keypad (UNO, static connected waveform) ====
// Pins: SDA=A4, SCL=A5 | PWM out (DDS): D9 (OC1A) | Keypad: Rows A0..A3, Cols D3..D6
// UI: Type 1..999 Hz, '*' clears, '#' applies. Preview redraws only on '#', not over time.
// Preview shows multiple cycles across 128 px, proportional to frequency, with CONNECTED trace.
//
// NOTE (hardware): D9/OC1A is your PWM source. Two RC stages -> LM358 buffer -> BNC.
// With R=3.9k & C=0.1uF (temporary), each pole ~408 Hz, so amplitude will droop at higher Hz.
// Later swap C to ~1.5 nF (or adjust R) for a flat 1..999 Hz passband.

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// ---------------- Config ----------------
#define SH1107_ADDR           0x3C   // set to 0x3D if your module uses it
#define RESERVED_TOP          16     // 2 text lines (y=0..15)
#define CYCLES_PER_50HZ       1      // ~1 cycle @ 50 Hz → ~10 cycles @ 500 Hz
#define MAX_CYCLES_ON_SCREEN  32     // cap so it stays readable

// ---------------- I2C helpers (chunked) ----------------
static inline void i2c_cmd(uint8_t c){
  Wire.beginTransmission(SH1107_ADDR);
  Wire.write(0x00); Wire.write(c);
  Wire.endTransmission();
}
static inline void i2c_data_block(const uint8_t* p, uint8_t n){
  Wire.beginTransmission(SH1107_ADDR);
  Wire.write(0x40);
  for(uint8_t i=0;i<n;i++) Wire.write(p[i]);
  Wire.endTransmission();
}
static inline void oled_write_page(uint8_t page, const uint8_t *buf){
  i2c_cmd(0xB0 | page); i2c_cmd(0x00); i2c_cmd(0x10);
  for(uint8_t off=0; off<128; off+=16) i2c_data_block(buf+off, 16);
}
static inline void oled_clear(){
  uint8_t z[16]; for(uint8_t i=0;i<16;i++) z[i]=0x00;
  for(uint8_t page=0; page<16; page++){
    i2c_cmd(0xB0 | page); i2c_cmd(0x00); i2c_cmd(0x10);
    for(uint8_t off=0; off<128; off+=16) i2c_data_block(z,16);
  }
}

// ---------------- OLED init ----------------
static void oled_init(){
  _delay_ms(50);
  i2c_cmd(0xAE);
  i2c_cmd(0xA8); i2c_cmd(0x7F);
  i2c_cmd(0xD3); i2c_cmd(0x00);
  i2c_cmd(0x40);
  i2c_cmd(0xA1);      // swap to 0xA0 if mirrored
  i2c_cmd(0xC8);      // swap to 0xC0 if upside-down
  i2c_cmd(0xDA); i2c_cmd(0x12);
  i2c_cmd(0x81); i2c_cmd(0x7F);
  i2c_cmd(0xD5); i2c_cmd(0x50);
  i2c_cmd(0xD9); i2c_cmd(0x22);
  i2c_cmd(0xDB); i2c_cmd(0x35);
  i2c_cmd(0xA4);
  i2c_cmd(0xA6);
  i2c_cmd(0xAF);
  oled_clear();
}

// ---------------- Tiny 5x7 font (PROGMEM) ----------------
static const uint8_t PROGMEM DIGITS[10][5] = {
  {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},
  {0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
  {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},{0x36,0x49,0x49,0x49,0x36},
  {0x06,0x49,0x49,0x29,0x1E}
};
static const uint8_t PROGMEM FCHR[5]={0x7F,0x09,0x09,0x09,0x01};
static const uint8_t PROGMEM HCHR[5]={0x7F,0x08,0x08,0x08,0x7F};
static const uint8_t PROGMEM ZCHR[5]={0x43,0x61,0x59,0x4D,0x47};
static const uint8_t PROGMEM COLN[5]={0x00,0x36,0x36,0x00,0x00};
static const uint8_t PROGMEM SCHR[5]={0x26,0x49,0x49,0x49,0x32};
static const uint8_t PROGMEM ECHR[5]={0x7F,0x49,0x49,0x49,0x41};
static const uint8_t PROGMEM TCHR[5]={0x01,0x01,0x7F,0x01,0x01};
static const uint8_t PROGMEM BLNK[5]={0,0,0,0,0};
static const uint8_t* pgm_font_ptr(char c){
  if(c>='0' && c<='9') return (const uint8_t*)DIGITS[c-'0'];
  switch(c){ case 'F': return FCHR; case 'H': return HCHR; case 'z': return ZCHR;
             case ':': return COLN; case 'S': return SCHR; case 'e': return ECHR;
             case 't': return TCHR; default: return BLNK; }
}
static void oled_print_line(uint8_t topY, const char* s){
  uint8_t page = topY >> 3;
  uint8_t buf[128]; for(uint8_t i=0;i<128;i++) buf[i]=0x00;
  uint8_t x=2;
  for(uint8_t i=0; s[i] && x+6<128; i++){
    const uint8_t* p = pgm_font_ptr(s[i]);
    for(uint8_t col=0; col<5; col++) buf[x++] = pgm_read_byte(p+col);
    buf[x++] = 0x00;
  }
  oled_write_page(page, buf);
}

// ---------------- Keypad (A0..A3 rows, D3..D6 cols) ----------------
#define KP_R_DDR   DDRC
#define KP_R_PORT  PORTC
#define KP_R0      PC0
#define KP_R1      PC1
#define KP_R2      PC2
#define KP_R3      PC3
#define KP_C_DDR   DDRD
#define KP_C_PORT  PORTD
#define KP_C_PIN   PIND
#define KP_C0      PD3
#define KP_C1      PD4
#define KP_C2      PD5
#define KP_C3      PD6

static void keypad_init(){
  KP_R_DDR  |= (1<<KP_R0)|(1<<KP_R1)|(1<<KP_R2)|(1<<KP_R3);
  KP_R_PORT |= (1<<KP_R0)|(1<<KP_R1)|(1<<KP_R2)|(1<<KP_R3);
  KP_C_DDR  &= ~((1<<KP_C0)|(1<<KP_C1)|(1<<KP_C2)|(1<<KP_C3));
  KP_C_PORT |=  (1<<KP_C0)|(1<<KP_C1)|(1<<KP_C2)|(1<<KP_C3);
}
static char keypad_get(){
  const uint8_t rows[4]={KP_R0,KP_R1,KP_R2,KP_R3};
  for(uint8_t r=0;r<4;r++){
    KP_R_PORT |=  (1<<KP_R0)|(1<<KP_R1)|(1<<KP_R2)|(1<<KP_R3);
    KP_R_PORT &= ~(1<<rows[r]);
    _delay_us(50);
    uint8_t c=(~KP_C_PIN)&((1<<KP_C0)|(1<<KP_C1)|(1<<KP_C2)|(1<<KP_C3));
    if(c){
      uint8_t col=(c&(1<<KP_C0))?0:(c&(1<<KP_C1))?1:(c&(1<<KP_C2))?2:3;
      static const char map[4][4]={
        {'1','2','3','A'},
        {'4','5','6','B'},
        {'7','8','9','C'},
        {'*','0','#','D'}
      };
      while((~KP_C_PIN)&((1<<KP_C0)|(1<<KP_C1)|(1<<KP_C2)|(1<<KP_C3)));
      _delay_ms(10);
      return map[r][col];
    }
  }
  return 0;
}

// ---------------- DDS + PWM (D9) ----------------
static volatile uint32_t phaseStep = 0;
static volatile uint8_t  amp_q8   = 220; // ~86% duty swing (adjust 180..255 to change amplitude)
static uint16_t freqHz = 1000, typed = 0;

// Sine LUT (0..255) in PROGMEM
static const uint8_t PROGMEM sineLUT[256] = {
  128,131,134,137,140,143,146,149,152,155,158,162,165,168,171,174,
  176,179,182,185,188,190,193,196,198,201,203,206,208,210,213,215,
  217,219,221,223,225,226,228,230,231,233,234,236,237,238,239,240,
  241,242,243,244,244,245,245,246,246,247,247,247,247,247,247,247,
  247,247,247,247,246,246,245,245,244,244,243,242,241,240,239,238,
  237,236,234,233,231,230,228,226,225,223,221,219,217,215,213,210,
  208,206,203,201,198,196,193,190,188,185,182,179,176,174,171,168,
  165,162,158,155,152,149,146,143,140,137,134,131,128,124,121,118,
  115,112,109,106,103,100,97,93,90,87,84,81,79,76,73,70,
  67,65,62,59,57,54,52,49,47,45,42,40,38,36,34,32,
  30,29,27,25,24,22,21,19,18,17,16,15,14,13,12,11,
  11,10,10,9,9,8,8,8,8,8,8,8,8,8,8,8,
  9,9,10,10,11,11,12,13,14,15,16,17,18,19,21,22,
  24,25,27,29,30,32,34,36,38,40,42,45,47,49,52,54,
  57,59,62,65,67,70,73,76,79,81,84,87,90,93,97,100,
  103,106,109,112,115,118,121,124
};

static void pwm_init(){
  DDRB |= (1<<PB1);                 // D9 as output
  TCCR1A = (1<<COM1A1) | (1<<WGM11);
  TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
  ICR1   = 255;                     // ~62.5 kHz PWM carrier at 16 MHz
  OCR1A  = 0;
}
static void timer2_init(){
  TCCR2A = (1<<WGM21);
  TCCR2B = (1<<CS21)|(1<<CS20);               // /32
  OCR2A  = (uint8_t)((F_CPU/(32UL*25000UL))-1);// ~19 for 25 kHz ISR
  TIMSK2 = (1<<OCIE2A);
}
static void dds_set_freq(uint16_t f){
  if(f<1) f=1; if(f>999) f=999;
  double step = ((double)f * 4294967296.0) / 25000.0; // Fs=25k
  uint32_t s = (uint32_t)(step+0.5);
  cli(); phaseStep = s; sei();
}
ISR(TIMER2_COMPA_vect){
  static uint32_t acc=0;
  acc += phaseStep;
  uint8_t idx = (uint8_t)(acc >> 24);
  uint8_t v   = pgm_read_byte(&sineLUT[idx]);
  uint16_t vv = ((uint16_t)v * amp_q8) >> 8;
  OCR1A = (uint8_t)vv;
}

// ---------------- Static preview draw (frequency-shaped, CONNECTED) ----------------
// Draw N cycles across 128 px, where N scales with 'f', with vertical connection
// between consecutive columns to avoid gaps at high frequency.
static void oled_draw_static_sine_for_freq(uint16_t f){
  // cycles ≈ CYCLES_PER_50HZ * f / 50, rounded; clamped 1..MAX_CYCLES_ON_SCREEN
  uint16_t cycles = (uint16_t)(((uint32_t)f * CYCLES_PER_50HZ + 25) / 50);
  if(cycles < 1) cycles = 1;
  if(cycles > MAX_CYCLES_ON_SCREEN) cycles = MAX_CYCLES_ON_SCREEN;

  const uint8_t firstPage = RESERVED_TOP >> 3;
  const uint8_t yRange    = 128 - RESERVED_TOP - 1;

  // Precompute y(x) for 0..127
  uint8_t yArr[128];
  for(uint8_t x=0; x<128; x++){
    // idx8 = (x * 2 * cycles) mod 256 (wrap by casting to 8-bit)
    uint16_t idx = (uint16_t)x * (uint16_t)(2 * cycles);
    uint8_t  v   = pgm_read_byte(&sineLUT[(uint8_t)idx]); // 0..255
    yArr[x] = RESERVED_TOP + (uint8_t)(((uint16_t)(255 - v) * yRange) / 255);
  }

  // Draw each page with connected vertical segments between consecutive columns
  for(uint8_t page=firstPage; page<16; page++){
    uint8_t buf[128]; for(uint8_t i=0;i<128;i++) buf[i]=0x00;
    uint8_t ylo = page*8, yhi = ylo+7;

    // first column
    uint8_t yPrev = yArr[0];
    if(yPrev>=ylo && yPrev<=yhi) buf[0] |= (1 << (yPrev & 7));

    for(uint8_t x=1; x<128; x++){
      uint8_t y = yArr[x];
      uint8_t a = (y < yPrev) ? y : yPrev;
      uint8_t b = (y > yPrev) ? y : yPrev;
      if(!(b < ylo || a > yhi)){
        uint8_t aa = (a < ylo) ? ylo : a;
        uint8_t bb = (b > yhi) ? yhi : b;
        for(uint8_t yy=aa; yy<=bb; yy++) buf[x] |= (1 << (yy & 7));
      }
      yPrev = y;
    }
    oled_write_page(page, buf);
  }
}

// ---------------- Text helpers ----------------
static void oled_print_freq(uint16_t f){
  char line[20]; uint8_t n=0; line[n++]='F'; line[n++]=':'; line[n++]=' ';
  char tmp[6]; uint8_t m=0; uint16_t t=f; do{ tmp[m++]='0'+(t%10); t/=10; } while(t);
  while(m--) line[n++]=tmp[m]; line[n++]=' '; line[n++]='H'; line[n++]='z'; line[n]=0;
  oled_print_line(0, line);
}
static void oled_print_set(uint16_t v){
  char line[20]; uint8_t n=0; line[n++]='S'; line[n++]='e'; line[n++]='t'; line[n++]=':'; line[n++]=' ';
  if(v==0) line[n++]='0'; else { char tmp[6]; uint8_t m=0; uint16_t t=v; do{ tmp[m++]='0'+(t%10); t/=10; } while(t);
                                      while(m--) line[n++]=tmp[m]; }
  line[n]=0; oled_print_line(8, line);
}

// ---------------- Setup / Loop ----------------
void setup(){
  Wire.begin();
  Wire.setClock(400000);
  oled_init();
  keypad_init();
  pwm_init();
  timer2_init();
  sei();

  dds_set_freq(freqHz);
  oled_print_freq(freqHz);
  oled_print_set(typed);
  oled_draw_static_sine_for_freq(freqHz);  // initial preview
}
void loop(){
  char k = keypad_get();
  if(!k) return;

  if(k>='0' && k<='9'){
    if(typed < 100) typed = typed*10 + (k-'0'); // up to 3 digits
    oled_print_set(typed);
  } else if(k=='*'){
    typed = 0; oled_print_set(typed);
  } else if(k=='#'){
    if(typed==0) typed=1; if(typed>999) typed=999;
    freqHz = typed; dds_set_freq(freqHz);
    oled_print_freq(freqHz);
    oled_draw_static_sine_for_freq(freqHz);     // redraw once; stays static
  }
}
