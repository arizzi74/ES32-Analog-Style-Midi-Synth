# ESP32 MIDI Synth Wiring (ESP32-WROOM-32 + 6N138 MIDI IN + PCM5102A I2S DAC)

This document contains **two complete wiring schematics** (in Markdown/ASCII) plus **pin maps** for an ESP32-WROOM-32 dev board:
- **MIDI IN** from a keyboard (MIDI OUT) using **DIN-5 + 6N138** into **ESP32 UART2 RX**
- **Audio OUT** using **PCM5102A I2S DAC** from ESP32 I2S

---

## 0) Quick overview (what connects to what)

- **Keyboard MIDI OUT → your DIN-5 MIDI IN**
- **DIN-5 → 6N138 optocoupler → ESP32 UART2 RX**
- **ESP32 I2S → PCM5102A (BCK/WS/DIN)**
- **ESP32 5V → PCM VIN**, **ESP32 GND → PCM GND**
- **PCM LOUT/ROUT → amplifier / line-in** (not a raw speaker)

---

## 1) MIDI IN schematic (DIN-5 → 6N138 → ESP32 UART2 RX)

### 1.1 ASCII schematic (recommended wiring)

```
Keyboard MIDI OUT  ──(MIDI cable)──>  Your MIDI IN

        DIN-5 (female)                       6N138 (DIP-8)                       ESP32
      ┌─────────────-─┐                  ┌─────────────-───┐                 ┌───────────┐
pin 4 o──R1 220Ω─────-+───────────────>  | 2  (A)          |                 |           |
                      |                  |                 |                 |           |
pin 5 o───────────────+───────────────>  | 3  (K)          |                 |           |
                      |                  |                 |                 |           |
(optional) D1 1N4148 reverse across LED  |                 |                 |           |
   D1 cathode → pin2(A), D1 anode → pin3(K)                |                 |           |
                                         | 8 (VCC) ── +5V  |<── ESP32 5V     |           |
                                         | 7 (GND) ── GND  |<── ESP32 GND    |           |
                                         |                 |                 |           |
                                         | 5 (VO) ─────────+───────────────> | GPIO16 RX |
                                         |                 |                 | (UART2 RX)|
                                         | 6 (VB)          |                 |           |
                                         └────────────-────┘                 └───────────┘

Output-side parts (strongly recommended with 6N138):
  R2 4.7k:  VO (pin5)  →  +3V3  (ESP32 3V3)      [pull-up; keeps RX 3.3V safe]
  R3 10k:   VB (pin6)  →  GND                   [improves turn-off/speed]
  R4 470–1k (optional): VB (pin6) ↔ VO (pin5)   [extra speed-up]
  C1 100nF: VCC (pin8) ↔ GND (pin7)             [decoupling; place close to U1]

DIN pin 2 (shield): leave NC (or chassis/shield only), NOT ESP32 GND.
```

### 1.2 MIDI UART settings
- **Baud:** 31250  
- **Format:** 8N1  
- **RX pin:** GPIO16 (example)  

---

## 2) I2S DAC schematic (ESP32 → PCM5102A)

### 2.1 ASCII wiring schematic (PCM5102A module)

Most PCM5102A boards expose: **VIN, GND, BCK, LCK/WS, DIN, SCK**, plus audio outputs.

```
ESP32 (I2S + power)                         PCM5102A DAC Module
┌──────────────────────────┐                ┌──────────────────────────────┐
| 5V  ─────────────────────+--------------> | VIN                          |
| GND ─────────────────────+--------------> | GND                          |
|                          |                |                              |
| GPIO14  (I2S BCK) ───────+--------------> | BCK    ⚠️ CHANGED FROM GPIO26|
| GPIO12  (I2S WS/LRCK) ───+--------------> | LCK/WS ⚠️ CHANGED FROM GPIO25|
| GPIO22  (I2S DATA OUT) ──+--------------> | DIN                          |
|                          |                |                              |
| GPIO25  (DAC out) ───────+─────────────┐  | SCK (MCLK in) ───────┐       |
|         Oscilloscope     |             |  |                      |       |
|                          |             |  |                      +-----> | GND  (tie low)
|                          |             |  |                              |
| 3V3 ─────────────────────+─────────────+> | XSMT (unmute)  (if exposed)  |
| GND ─────────────────────+--------------> | FMT  (I2S mode) (if exposed) |
└──────────────────────────┘                └──────────────────────────────┘

Clock:
  SCK (MCLK input) → GND   (3-wire PLL mode, no MCLK from ESP32)

Config strapping (only if your module exposes these pins):
  FMT  → GND   (select I2S format)
  XSMT → 3V3   (unmuted)

Audio out:
  LOUT / ROUT → amplifier input or line-in (NOT a raw speaker)
  AGND        → audio ground (often same as module GND)
```

### 2.2 Strap resistor suggestions (optional)
If you prefer straps via resistors instead of direct wires:
- **FMT pulldown:** 10k to GND  
- **XSMT pullup:** 10k to 3.3V  

---

## 3) Pin maps

### 3.1 ESP32 pin map (UPDATED - GPIO25 freed for DAC)
| Function | ESP32 Pin | Notes |
|---|---|---|
| MIDI UART2 RX | GPIO16 | |
| I2S BCK | **GPIO14** | ⚠️ Changed from GPIO26 |
| I2S WS/LRCK | **GPIO12** | ⚠️ Changed from GPIO25 |
| I2S DATA OUT | GPIO22 | Unchanged |
| **DAC Output (Oscilloscope)** | **GPIO25** | 8-bit analog, 0-3.3V |
| 5V (to PCM VIN + 6N138 VCC) | 5V pin (USB 5V) | |
| 3V3 (pull-up + XSMT) | 3V3 pin | |
| GND | any GND pin | |

### 3.2 PCM5102A module pin map (UPDATED)
| PCM5102A module pin | Connect to | Notes |
|---|---|---|
| VIN | ESP32 5V | |
| GND | ESP32 GND | |
| BCK | **ESP32 GPIO14** (I2S BCK) | ⚠️ Changed from GPIO26 |
| LCK/WS | **ESP32 GPIO12** (I2S WS/LRCK) | ⚠️ Changed from GPIO25 |
| DIN | ESP32 GPIO22 (I2S DATA OUT) | Unchanged |
| SCK | GND (tie low for 3-wire PLL mode) | |
| FMT (if exposed) | GND (I2S mode select) | |
| XSMT (if exposed) | 3V3 (unmute) | |
| LOUT/ROUT | amplifier / line-in | |
| AGND | audio ground (often same as GND) | |

### 3.3 6N138 pin map (DIP-8)
| 6N138 pin | Name | Connect to |
|---|---|---|
| 2 | A (LED anode) | DIN pin 4 via R1 220Ω |
| 3 | K (LED cathode) | DIN pin 5 |
| 5 | VO | ESP32 GPIO16 (RX) + pull-up R2 to 3V3 |
| 6 | VB | R3 10k to GND (and optional R4 to VO) |
| 7 | GND | ESP32 GND |
| 8 | VCC | ESP32 5V |
| 1,4 | NC | leave unconnected |

---

## 4) Parts list (BOM)

### MIDI IN (6N138)
- U1: 6N138
- R1: 220Ω
- R2: 4.7kΩ (VO pull-up to 3V3)
- R3: 10kΩ (VB to GND)
- R4: 470Ω–1kΩ (optional speed-up, VB ↔ VO)
- C1: 100nF ceramic (decoupling, place close to U1)
- D1: 1N4148 (optional reverse protection)
- DIN-5 female jack

### Audio (PCM5102A)
- PCM5102A I2S DAC module
- Amplifier / powered speakers / line-in destination

---

## 5) Quick sanity checklist

- **VO pull-up goes to 3.3V**, not 5V (protects ESP32 RX).
- **ESP32 GND and PCM GND must be common**.
- **Tie PCM SCK to GND** for 3-wire mode (no MCLK).
- **LOUT/ROUT are line-level**: use an amplifier or line-in (do not drive a bare speaker).
- DIN pin 2 (shield) usually **not** tied to ESP32 GND (leave NC or chassis).

---

## 6) Notes (optional but helpful)

- If MIDI feels unreliable with 6N138, add the optional **R4** (VB↔VO) and/or use a stronger pull-up (e.g., 2.2k–4.7k).
- Keep I2S and MIDI wiring short and tidy; add decoupling caps close to ICs.

