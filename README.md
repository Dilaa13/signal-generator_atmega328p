# 🎵 Atmega328p DDS Sine Wave Generator with OLED Preview & Keypad Input

This project implements a **Direct Digital Synthesis (DDS)** sine wave generator on an **ATmega328P**.  
It combines:

- A **4×4 keypad** for frequency input (1–999 Hz).
- A **128×128 I²C OLED display (SH1107 controller)** to show frequency and a static sine preview.
- A **PWM DDS engine** running on pin **D9 (OC1A)**, smoothed with a **two-stage RC filter** and buffered using an **LM358 op-amp**.
- A **BNC output** for connecting to test instruments like oscilloscopes, audio circuits, or signal-processing experiments.

---

## ✨ Features
- **Keypad input (A0–A3 rows, D3–D6 cols)**  
  Type digits → frequency buffer.  
  `*` clears buffer, `#` applies frequency.

- **OLED display (I²C, SH1107 128×128, SDA=A4, SCL=A5)**  
  - Displays:
    - Current frequency (`F: #### Hz`)  
    - Pending typed value (`Set: ###`)  
    - A static sine preview scaled to the chosen frequency.  
  - Clear, **1-pixel thick connected waveform** rendering for readability.

- **DDS PWM signal (D9 / OC1A)**  
  - 62.5 kHz PWM carrier modulated with sine duty cycle.  
  - Frequency range: **1–999 Hz**.  
  - Amplitude adjustable in code via `amp_q8` variable.

- **Analog output via RC filter + LM358**  
  - Two-stage RC low-pass filter attenuates PWM carrier.  
  - Unity-gain buffer with LM358 provides clean sine at BNC jack.  
  - Designed for easy connection to oscilloscope or audio stages.

---

## 🔧 Hardware Requirements
- **Atmega328p Uno (ATmega328P)**
- **128×128 I²C OLED** (SH1107 controller, addr `0x3C` or `0x3D`)
- **4×4 Keypad**
- **Op-amp buffer:** LM358 (dual, DIP-8)
- **BNC connector** (panel mount)
- **RC filter parts**:
  - R1 = 3.9 kΩ  
  - R2 = 3.9 kΩ  
  - C1 = 1.5 nF (1–2.2 nF OK)  
  - C2 = 1.5 nF (1–2.2 nF OK)  
  *(Temporary build used 0.1 µF caps, but these attenuate higher frequencies too much. For 1–999 Hz flat response, use ~1.5 nF.)*

---

## ⚡ Pin Mapping (Atmega328p Uno → ATmega328P)
| Function              | Atmega328p Pin | ATmega328P Pin |
|-----------------------|-------------|----------------|
| OLED SDA              | A4          | Pin 27 (PC4)   |
| OLED SCL              | A5          | Pin 28 (PC5)   |
| Keypad Row 0          | A0          | Pin 23 (PC0)   |
| Keypad Row 1          | A1          | Pin 24 (PC1)   |
| Keypad Row 2          | A2          | Pin 25 (PC2)   |
| Keypad Row 3          | A3          | Pin 26 (PC3)   |
| Keypad Col 0          | D3          | Pin 5  (PD3)   |
| Keypad Col 1          | D4          | Pin 6  (PD4)   |
| Keypad Col 2          | D5          | Pin 11 (PD5)   |
| Keypad Col 3          | D6          | Pin 12 (PD6)   |
| DDS PWM Output        | D9 (OC1A)   | Pin 15 (PB1)   |
| GND                   | GND         | Pin 8, 22      |
| VCC                   | 5V          | Pin 7, 20      |

---

## 🖥️ Software Overview
### Core Components
1. **OLED Driver (I²C, SH1107)**  
   - Uses chunked I²C transfers (≤16 bytes) to avoid Wire buffer overflows.  
   - Static waveform preview is drawn with connected lines (no flicker, no animation).

2. **Keypad Scanner**  
   - Rows set as outputs, columns read with pull-ups.  
   - Scans row-by-row, detects pressed key, maps to digit/command.

3. **DDS Engine**  
   - 25 kHz sample ISR (Timer2).  
   - Phase accumulator increments with `phaseStep`.  
   - High byte indexes into 256-entry sine LUT (0–255).  
   - Scaled amplitude sets PWM duty cycle via OCR1A.  
   - Timer1 runs at 62.5 kHz PWM.

4. **Waveform Preview**  
   - Number of cycles across 128 px depends on frequency.  
   - At low frequencies: fewer cycles (larger wave).  
   - At high frequencies: more cycles, capped at 32 per screen.  
   - Connected vertical lines ensure continuous look.

---

## 📊 Example Workflow
1. **Power up**  
   - OLED shows:  
     `F: 1000 Hz`  
     `Set: 0`  
     Plus static sine preview.

2. **Enter new frequency**  
   - Type digits on keypad (e.g., `5 0 0`).  
   - OLED shows `Set: 500`.

3. **Apply frequency**  
   - Press `#`.  
   - OLED updates `F: 500 Hz`.  
   - Preview redraws with cycles matching 500 Hz.  
   - PWM/DDS output on **D9** changes to 500 Hz sine.

4. **Clear input**  
   - Press `*` to reset `Set: 0`.

---

## 🛠️ Building the Analog Output Stage
**PWM on D9** → **R1 (3.9k) → Node V1**  
- C1 (1.5nF) → GND  

**V1 → R2 (3.9k) → Node V2**  
- C2 (1.5nF) → GND  

**V2 → LM358 buffer (unity gain)**  
- Pin 3 (IN+) = V2  
- Pin 2 (IN–) shorted to Pin 1 (OUT)  
- OUT → 50 Ω resistor → BNC center  
- BNC shell → GND  

---

## 📷 Demo
*(Insert oscilloscope screenshots of clean sine waves at different frequencies and OLED photos here)*

---

## 🚀 Future Improvements
- Adjustable amplitude control via keypad.  
- Multiple waveforms (square, triangle, sawtooth) selectable from the keypad.  
- Store last frequency in EEPROM.  
- Optional rotary encoder for quick tuning.

---

## 📄 License
MIT License © 2025  

Feel free to fork, modify, and use in your own projects. Attribution is appreciated! 🙌

---

## 🙌 Credits
This project was built by experimenting with:  
- AVR registers and timers  
- SH1107 OLED datasheet  
- DDS sine generation concepts  
- Practical RC filter design and op-amp buffering
