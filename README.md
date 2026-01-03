# ESP32 MIDI Synthesizer

A professional-grade duophonic MIDI synthesizer for ESP32 with PolyBLEP oscillators, analog-modeled filters, and sub/unison thickness control.

## Features

- 🎹 **Duophonic Paraphony:** 2 simultaneous notes with shared envelopes/filters
- 🎚️ **Rich Synthesis:**
  - PolyBLEP anti-aliased oscillators (saw, square, triangle, sine)
  - Sub-oscillator (-1 octave square wave)
  - Unison detune (8 cents for thickness)
  - Analog drift emulation (per-oscillator pitch instability)
- 🎛️ **Flexible Modulation:**
  - Dual ADSR envelopes (amplitude + filter)
  - LFO with 4 modes (filter wobble, vibrato, sample & hold)
  - State-variable filters (low-pass + high-pass)
  - Filter drive/saturation (1-6×)
- 💾 **NVS Persistence:** All 12 controller settings saved to flash
- 📊 **Oscilloscope Output:** Real-time waveform on GPIO25 (8-bit DAC, 44.1 kHz)
- ⚡ **Ultra-Low Latency:** 0.73ms audio latency, <1ms MIDI latency

---

## Hardware Requirements

### **Components**
- ESP32-WROOM-32 dev board
- PCM5102A I2S DAC module
- 6N138 optocoupler (for MIDI input)
- DIN-5 female MIDI jack
- Resistors: 220Ω, 4.7kΩ, 10kΩ (see BOM below)
- 100nF ceramic capacitor

### **Bill of Materials (BOM)**

#### MIDI Input Circuit (6N138)
| Component | Value | Purpose |
|-----------|-------|---------|
| U1 | 6N138 optocoupler | MIDI isolation |
| R1 | 220Ω | MIDI current limit |
| R2 | 4.7kΩ | VO pull-up to 3.3V |
| R3 | 10kΩ | VB to GND (speed-up) |
| R4 | 470Ω-1kΩ (optional) | VB↔VO speed-up |
| C1 | 100nF ceramic | Decoupling |
| D1 | 1N4148 (optional) | Reverse protection |
| J1 | DIN-5 female jack | MIDI input connector |

#### Audio Output
| Component | Type | Notes |
|-----------|------|-------|
| PCM5102A | I2S DAC module | 32-bit stereo, 384kHz capable |

---

## Pin Configuration

### **ESP32 → PCM5102A (I2S Audio)**
| ESP32 Pin | PCM5102A Pin | Signal |
|-----------|--------------|--------|
| **GPIO14** | BCK | Bit clock |
| **GPIO12** | LCK/WS | Word select |
| **GPIO22** | DIN | Data |
| 5V | VIN | Power |
| GND | GND | Ground |
| - | SCK | Tie to GND (3-wire mode) |
| 3.3V | XSMT (if exposed) | Unmute |
| GND | FMT (if exposed) | I2S format select |

**⚠️ Note:** Pins have been remapped from common configurations to free GPIO25 for DAC output.

### **ESP32 ← 6N138 (MIDI Input)**
| ESP32 Pin | 6N138 Pin | Signal | Notes |
|-----------|-----------|--------|-------|
| **GPIO16** | Pin 5 (VO) | UART RX | With 4.7kΩ pull-up to 3.3V |
| 5V | Pin 8 (VCC) | Power | |
| GND | Pin 7 (GND) | Ground | |
| 3.3V | - | Pull-up rail | Via R2 to VO |

### **6N138 ← MIDI DIN-5**
| DIN-5 Pin | 6N138 Pin | Via | Signal |
|-----------|-----------|-----|--------|
| Pin 4 | Pin 2 (A) | R1 220Ω | Current |
| Pin 5 | Pin 3 (K) | Direct | Return |
| Pin 2 | - | Leave NC | Shield (optional) |

### **DAC Oscilloscope Output**
| ESP32 Pin | Output | Specs |
|-----------|--------|-------|
| **GPIO25** | Analog audio | 0-3.3V, 8-bit, 44.1kHz |
| GND | Ground | Scope ground reference |

---

## MIDI Controller Reference

### **Quick Reference Table**
| CC# | Parameter | Range | Default | Function |
|-----|-----------|-------|---------|----------|
| **16** | Analog Drift | 0-2 cents | 0 | Per-oscillator pitch instability |
| **17** | Env Release | 0-10s | 100ms | Release time |
| **18** | LFO Rate | 0.1-20 Hz | 2 Hz | Modulation speed |
| **19** | Filter Drive | 1-6× | 1× | Pre-filter saturation |
| **71** | LP Resonance | 0-1.0 | 0.7 | Low-pass Q factor |
| **74** | LP Cutoff | 100-10k Hz | 1000 Hz | Low-pass brightness |
| **76** | HP Cutoff | 20-2k Hz | 20 Hz | High-pass thinning |
| **77** | Thickness | 0-127 | 0 | Sub + unison mix |
| **82** | Env Attack | 0-5s | 10ms | Attack time |
| **83** | Env Decay | 0-10s | 50ms | Decay time |
| **85** | Env Sustain | 0-1.0 | 0.7 | Sustain level |
| **93** | LFO Mode | 0-3 | 0 | PWM/Filter/Vibrato/S&H |

**All parameters auto-save to NVS flash 2 seconds after last change.**

### **CC77 Thickness Behavior**
| Value | Effect |
|-------|--------|
| **0** | Clean main oscillator only |
| **1-57** | Main + sub bass (0-29% mix) |
| **58-127** | Main + sub + detuned unison (full thickness) |

### **CC93 LFO Modes**
| Value | Mode | Waveform | Destination | Depth |
|-------|------|----------|-------------|-------|
| **0-31** | PWM | Triangle | (disabled) | - |
| **32-63** | Filter Wobble | Sine | LP Cutoff | ±3kHz |
| **64-95** | Vibrato | Sine | Pitch | ±0.05 semitones |
| **96-127** | Sample & Hold | S&H | LP Cutoff | ±4kHz |

---

## Wiring Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32-WROOM-32                       │
│                                                             │
│  [MIDI Input]                                               │
│  GPIO16 (RX) ◄──[R2 4.7k to 3V3]──┬─── 6N138 Pin 5 (VO)     │
│                                    │                        │
│  5V ──────────────────────────────┼─── 6N138 Pin 8 (VCC)    │
│  GND ─────────────────────────────┼─── 6N138 Pin 7 (GND)    │
│                                    │                        │
│                              [R3 10k to GND]                │
│                                                             │
│  [I2S Audio Output]                                         │
│  GPIO14 ─────────────────────────────► PCM5102 BCK          │
│  GPIO12 ─────────────────────────────► PCM5102 LCK          │
│  GPIO22 ─────────────────────────────► PCM5102 DIN          │
│  5V ──────────────────────────────────► PCM5102 VIN         │
│  GND ─────────────────────────────────► PCM5102 GND         │
│                                                             │
│  [Oscilloscope Output]                                      │
│  GPIO25 ─────────────────────────────► Scope Probe          │
│  GND ────────────────────────────────► Scope Ground         │
└─────────────────────────────────────────────────────────────┘

   6N138 Optocoupler                       MIDI DIN-5 (Female)
   ┌──────────────┐                         ┌─────────┐
   │ 2 (A) ───────┼────[R1 220Ω]────────-─  │ Pin 4   │
   │              │                         │         │
   │ 3 (K) ───────┼───────────────────────  │ Pin 5   │
   │              │                         │         │
   │ 5 (VO) ──────┼──► ESP32 GPIO16         │ (Pin 2  │
   │              │    (with R2 pull-up)    │ = NC)   │
   │ 6 (VB) ──────┼──[R3 10k]──GND          └─────────┘
   │              │
   │ 7 (GND) ─────┼──GND
   │ 8 (VCC) ─────┼──5V
   │              │
   │ 1,4 (NC)     │
   └──────────────┘
```

---

## Quick Start

### **1. Build & Flash**
```bash
cd esp32synth
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### **2. Wiring Checklist**
- ✅ PCM5102 SCK tied to GND (3-wire mode)
- ✅ 6N138 VO pull-up to **3.3V** (not 5V!)
- ✅ All grounds connected (ESP32, PCM5102, 6N138, MIDI)
- ✅ MIDI DIN pin 2 (shield) left unconnected or chassis only

### **3. Test**
- Send MIDI note from controller
- Hear audio from PCM5102 line output
- Observe waveform on oscilloscope (GPIO25)

---

## Audio Output

### **PCM5102A Connections**
- **LOUT/ROUT** → Amplifier input or line-in
- **DO NOT** connect directly to speaker (line-level only!)

### **Recommended Amplifiers**
- Powered studio monitors
- Headphone amplifier
- Mixer line input
- Audio interface

---

## Specifications

| Parameter | Value |
|-----------|-------|
| **Sample Rate** | 44.1 kHz |
| **Bit Depth** | 16-bit |
| **Polyphony** | 2 voices (duophonic paraphony) |
| **Audio Latency** | 0.73 ms (32 samples) |
| **MIDI Latency** | <1 ms (byte-by-byte) |
| **MIDI Baud Rate** | 31250 |
| **CPU Usage** | ~85% (core 1 audio task) |
| **Flash Size** | 256 KB (75% free) |
| **RAM Usage** | ~8 KB (stack + heap) |

---

## Signal Flow

```
MIDI In (31250 baud)
    ↓
UART RX Task (GPIO16, byte-by-byte)
    ↓
MIDI Processing Task (note on/off, CC)
    ↓
┌────────────────────────────────────────────────────┐
│ Audio Generation Task (44.1 kHz, Core 1)           │
│                                                    │
│  Oscillators ──┬─► Main (PolyBLEP: saw/sq/tri)     │
│                ├─► Sub (-1 octave square)          │
│                └─► Unison (8¢ detune)              │
│         ↓                                          │
│  Mix & Average (paraphonic)                        │
│         ↓                                          │
│  High-Pass Filter (CC76)                           │
│         ↓                                          │
│  Drive & Saturation (CC19)                         │
│         ↓                                          │
│  Low-Pass Filter (CC74/71, envelope + LFO)         │
│         ↓                                          │
│  Amplitude Envelope (CC82/83/85/17)                │
│         ↓                                          │
│  Soft Clip                                         │
│         ↓                                          │
│  ┌─────────┬──────────┐                            │
│  ↓         ↓          ↓                            │
│ I2S     DAC      Stereo                            │
│(PCM5102) (GPIO25) (16-bit)                         │
└────────────────────────────────────────────────────┘
    ↓         ↓
  Audio   Oscilloscope
```

---

## Development

### **Prerequisites**
- ESP-IDF v5.4 or later
- ESP32-WROOM-32 dev board
- USB cable for programming

### **Build System**
- CMake-based (ESP-IDF standard)
- Component: `esp32synth/main`

### **Code Structure**
- **Single file:** `main/esp32synth.c` (~1300 lines)
- **Documentation:**
  - `CODE.md` - Detailed code architecture
  - `MIDI_CONTROLLERS.md` - MIDI CC reference
  - `ESP32_MIDI_PCM5102A_6N138_WIRING.md` - Hardware guide

---

## Presets

### **Classic Analog Bass**
```
CC74=40, CC71=80, CC77=60, CC82=5, CC17=30, CC19=50
```

### **Evolving Pad**
```
CC74=70, CC71=50, CC77=100, CC82=80, CC85=120, CC17=100
CC18=20, CC93=50
```

### **Plucky Lead**
```
CC74=90, CC71=60, CC77=30, CC82=0, CC83=40, CC85=30, CC17=20
```

### **Distorted Wobble Bass**
```
CC74=50, CC71=100, CC77=80, CC19=100, CC18=90, CC93=110, CC16=60
```

---

## Troubleshooting

### **No Audio**
- Check PCM5102 wiring (BCK=GPIO14, LCK=GPIO12, DOUT=GPIO22)
- Verify SCK tied to GND
- Test with amplifier/headphones (not bare speaker)

### **No MIDI Response**
- Check 6N138 wiring (VO to GPIO16 with 4.7kΩ to 3.3V)
- Verify MIDI cable orientation (OUT → IN)
- Monitor serial console for MIDI messages

### **Distorted Sound**
- Reduce CC19 (filter drive)
- Lower CC77 (thickness)
- Check PCM5102 power supply (clean 5V)

### **Watchdog Timeout**
- Reduce CC77 below 80 if using CC16 (drift)
- Disable logging (already suppressed except errors)

---

## Performance Notes

### **CPU Optimizations**
1. **Pre-computed constants:** Sine table, detune ratio
2. **Per-buffer calculations:** LFO, drift (not per-sample)
3. **Conditional processing:** Sub/unison only if thickness > 0
4. **Unison threshold:** Only enabled above CC77=57 (~32%)
5. **Small buffers:** 16 samples for fast task yielding

### **Known Limitations**
- 2-voice maximum (CPU constraint)
- No polyphony (paraphonic only)
- Fixed 44.1 kHz sample rate
- 8-bit DAC resolution (oscilloscope output)

---

## License

This project is open-source. See LICENSE file for details.

---

## Credits

**Developed by:** Antonio Vivace
**Platform:** ESP-IDF v5.4+
**Hardware:** ESP32-WROOM-32 + PCM5102A + 6N138

---

## Additional Resources

- **[CODE.md](CODE.md)** - Detailed code architecture and structure
- **[MIDI_CONTROLLERS.md](MIDI_CONTROLLERS.md)** - Complete MIDI controller reference
- **[ESP32_MIDI_PCM5102A_6N138_WIRING.md](ESP32_MIDI_PCM5102A_6N138_WIRING.md)** - Hardware wiring guide

---

## Version

**v1.0** - Initial release (January 2026)
