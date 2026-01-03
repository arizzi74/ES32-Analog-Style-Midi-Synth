# ESP32 MIDI Synthesizer - Controller Reference

## Overview
This synthesizer supports duophonic paraphony (2 simultaneous notes with shared envelopes and filters). All controller values are automatically saved to NVS flash 2 seconds after the last change and restored on boot.

---

## Synthesis Architecture

**Signal Flow:**
```
MIDI Note → Oscillators (Main + Sub + Unison) → High-Pass Filter →
Drive/Saturation → Low-Pass Filter (with envelope modulation) →
Amplitude Envelope → Output
```

**Voice Architecture:**
- **Paraphonic:** Up to 2 notes with independent oscillators
- **Shared:** Single envelope and filter for all voices
- **Per-Note:** Each note has main + sub + unison oscillators with analog drift

---

## MIDI Controllers

### Oscillator & Thickness

#### **CC77 - Thickness (Sub + Unison Mix)**
- **Range:** 0-127
- **Function:** Blends in sub-oscillator and detuned unison for analog warmth
- **Default:** 0 (clean main oscillator only)

**Behavior:**
- **0:** Main oscillator only (cleanest sound)
- **1-57:** Main + sub bass (-1 octave square wave)
  - Sub mix increases from 0% to 29%
  - Adds warm low-end foundation
- **58-127:** Main + sub + detuned unison
  - Unison: 8 cents detune for chorus/thickness
  - Full thickness at 127: Main + 35% sub + 50% unison

**Use Cases:**
- CC77 = 0: Clean leads, precise bass lines
- CC77 = 40: Warm bass with subtle sub
- CC77 = 80: Thick pads and evolving textures
- CC77 = 127: Maximum analog character

**Notes:**
- Uses squared curve for smooth control at low values
- Phases maintain continuity when sweeping (no clicks)
- Saved to NVS flash

---

#### **CC16 - Analog Drift Depth**
- **Range:** 0-127 (maps to 0.00-2.00 cents)
- **Function:** Per-oscillator pitch drift for analog instability
- **Default:** 0 (perfect tuning)

**Behavior:**
- Each oscillator has independent smooth random drift
- Drift is slow (< 10 Hz) for organic pitch variation
- Applied equally to main, sub, and unison oscillators
- Creates "chorusing" effect with multiple notes

**Use Cases:**
- CC16 = 0: Digital-perfect tuning
- CC16 = 32: Subtle warmth (0.5 cents)
- CC16 = 64: Vintage analog character (1.0 cents)
- CC16 = 127: Maximum drift (2.0 cents, obvious wobble)

**Technical:**
- Uses per-oscillator xorshift32 PRNG with low-pass filtering
- Update rate: ~2700 Hz (once per audio buffer)
- Saved to NVS flash

---

### Filter Section

#### **CC74 - Low-Pass Filter Cutoff**
- **Range:** 0-127 (maps to 100-10000 Hz)
- **Function:** Sets LP filter brightness
- **Default:** 1000 Hz

**Behavior:**
- Controls the main low-pass filter cutoff frequency
- Modulated by filter envelope (CC82/83/85/17)
- Modulated by LFO when in filter modes (CC93)
- Logarithmic mapping for musical response

**Use Cases:**
- CC74 = 20: Dark, muffled bass tones
- CC74 = 64: Warm, balanced timbre
- CC74 = 100: Bright, open sound
- CC74 = 127: Maximum brightness

**Notes:**
- State-variable filter design
- Envelope adds up to 5000 Hz modulation
- Saved to NVS flash

---

#### **CC71 - Low-Pass Filter Resonance**
- **Range:** 0-127 (maps to 0.0-1.0)
- **Function:** Controls LP filter resonance/Q
- **Default:** 0.7

**Behavior:**
- Increases emphasis at cutoff frequency
- High values create self-oscillation peak
- Interacts with envelope modulation

**Use Cases:**
- CC71 = 0: Gentle roll-off (no resonance)
- CC71 = 64: Moderate resonance (classic synth)
- CC71 = 100: High resonance (squelchy)
- CC71 = 127: Maximum resonance (vocal quality)

**Notes:**
- Can create whistling/ringing at high values
- Saved to NVS flash

---

#### **CC76 - High-Pass Filter Cutoff**
- **Range:** 0-127 (maps to 20-2000 Hz)
- **Function:** Removes low-frequency content
- **Default:** 20 Hz (minimal effect)

**Behavior:**
- Applied before drive and low-pass filter
- Useful for thinning out muddy bass
- Can be used creatively for filter sweeps

**Use Cases:**
- CC76 = 0: Full low-end (20 Hz)
- CC76 = 40: Subtle bass reduction
- CC76 = 80: Telephone/radio effect
- CC76 = 127: Thin, treble-only (2000 Hz)

**Notes:**
- State-variable filter, fixed resonance (0.5)
- Saved to NVS flash

---

#### **CC19 - Filter Drive**
- **Range:** 0-127 (maps to 1.0-6.0×)
- **Function:** Pre-filter saturation/distortion
- **Default:** 1.0 (clean)

**Behavior:**
- Multiplies signal before low-pass filter
- Soft clipping creates harmonic distortion
- Interacts with filter cutoff/resonance

**Use Cases:**
- CC19 = 0: Clean, linear (1.0×)
- CC19 = 42: Subtle warmth (2.0×)
- CC19 = 85: Aggressive saturation (4.0×)
- CC19 = 127: Maximum distortion (6.0×)

**Notes:**
- Uses soft_clip() for analog-style saturation
- Saved to NVS flash

---

### LFO (Low-Frequency Oscillator)

#### **CC18 - LFO Rate**
- **Range:** 0-127 (maps to 0.1-20.0 Hz)
- **Function:** Sets LFO speed
- **Default:** 2.0 Hz

**Behavior:**
- Controls LFO frequency for modulation
- Destination determined by CC93 (LFO mode)
- Smoothed to prevent zipper noise

**Use Cases:**
- CC18 = 13: Slow evolving (0.1 Hz, 10-second cycle)
- CC18 = 64: Moderate rate (5 Hz)
- CC18 = 100: Fast wobble (12 Hz)
- CC18 = 127: Maximum rate (20 Hz, near audio rate)

**Notes:**
- Shared by all voices (paraphonic)
- Saved to NVS flash

---

#### **CC93 - LFO Mode**
- **Range:** 0-127 (4 modes)
- **Function:** Selects LFO destination
- **Default:** 0 (PWM - currently disabled)

**Modes:**

| CC93 Value | Mode | Waveform | Destination | Depth | Effect |
|------------|------|----------|-------------|-------|--------|
| **0-31** | PWM Modulation | Triangle | (disabled) | - | CPU optimization |
| **32-63** | Filter Wobble | Sine | LP Cutoff | ±3000 Hz | Smooth filter sweep |
| **64-95** | Vibrato | Sine | Pitch | ±0.05 semitones | Smooth pitch modulation |
| **96-127** | Sample & Hold | S&H | LP Cutoff | ±4000 Hz | Random stepped filter |

**Use Cases:**
- **Filter Wobble (CC93=50):** Classic wah/dubstep wobble
- **Vibrato (CC93=80):** Vocal-like pitch variation
- **Sample & Hold (CC93=110):** Random stepped filter for glitchy/alien sounds

**Notes:**
- Mode 0-31 (PWM) is disabled to reduce CPU load
- Saved to NVS flash

---

### Envelope (ADSR)

All envelope parameters are shared across both oscillators (paraphonic).

#### **CC82 - Envelope Attack**
- **Range:** 0-127 (maps to 0-5000 ms)
- **Function:** Attack time (note on → peak)
- **Default:** 10 ms

**Behavior:**
- Linear ramp from 0 to 1.0
- Applied to both amplitude and filter envelopes
- Fast attack = percussive, slow attack = pad-like

**Use Cases:**
- CC82 = 0: Instant attack (0 ms, plucky/percussive)
- CC82 = 25: Fast attack (1000 ms, responsive)
- CC82 = 64: Medium attack (2500 ms, pad-like)
- CC82 = 127: Maximum attack (5000 ms, slow fade-in)

**Notes:**
- Updates smoothly during playback (no clicks)
- Saved to NVS flash

---

#### **CC83 - Envelope Decay**
- **Range:** 0-127 (maps to 0-10000 ms)
- **Function:** Decay time (peak → sustain)
- **Default:** 50 ms

**Behavior:**
- Exponential decay from peak to sustain level
- Only audible if sustain < 1.0

**Use Cases:**
- CC83 = 0: Instant decay (0 ms, sustain immediately)
- CC83 = 32: Fast decay (2500 ms, plucky)
- CC83 = 64: Medium decay (5000 ms, balanced)
- CC83 = 127: Slow decay (10000 ms, evolving)

**Notes:**
- Updates smoothly during playback
- Saved to NVS flash

---

#### **CC85 - Envelope Sustain**
- **Range:** 0-127 (maps to 0.0-1.0)
- **Function:** Sustain level while note held
- **Default:** 0.7

**Behavior:**
- Level maintained while key is held
- 0.0 = silent, 1.0 = full volume

**Use Cases:**
- CC85 = 0: No sustain (organ-like envelope)
- CC85 = 64: Half sustain (balanced)
- CC85 = 100: High sustain (pad-like)
- CC85 = 127: Full sustain (organ/drone)

**Notes:**
- Updates smoothly during playback
- Saved to NVS flash

---

#### **CC17 - Envelope Release**
- **Range:** 0-127 (maps to 0-10000 ms)
- **Function:** Release time (note off → silence)
- **Default:** 100 ms

**Behavior:**
- Exponential decay from current level to 0
- Longer release = smoother note transitions

**Use Cases:**
- CC17 = 0: Instant release (0 ms, staccato)
- CC17 = 32: Fast release (2500 ms, plucky)
- CC17 = 64: Medium release (5000 ms, smooth)
- CC17 = 127: Maximum release (10000 ms, long tail)

**Notes:**
- Oscillators remain active during release (paraphonic)
- Updates smoothly during playback
- Saved to NVS flash

---

### Filter Envelope Modulation

The filter envelope uses the same ADSR parameters (CC82/83/85/17) but modulates the LP filter cutoff instead of amplitude.

**Behavior:**
- Envelope value × 5000 Hz is added to LP cutoff (CC74)
- Creates classic "wah" opening on note attack
- Amount controlled by internal `filter_env_amount` (0.5 = 2500 Hz max)

**Example:**
- CC74 = 64 (base cutoff ~1000 Hz)
- Attack: Filter opens to ~3500 Hz
- Decay: Filter closes to sustain level
- Release: Filter returns to base cutoff

---

## Other MIDI Messages

### **Note On/Off**
- **Range:** MIDI notes 0-127
- **Paraphonic Behavior:**
  - First note: Triggers envelopes
  - Second note: Adds oscillator, envelopes continue
  - Note off: Removes oscillator
  - Last note off: Triggers envelope release

**Frequency Range:** ~8 Hz (MIDI 0) to ~12.5 kHz (MIDI 127)

---

### **Velocity**
- **Range:** 0-127
- **Function:** Scales amplitude envelope output
- **Behavior:**
  - Velocity 0 = silent
  - Velocity 127 = full volume
  - Linear scaling

---

### **Pitch Bend**
- **Range:** 0-16383 (14-bit), center = 8192
- **Function:** Real-time pitch modulation
- **Bend Range:** ±2 semitones
- **Behavior:**
  - Applied to all active oscillators (main, sub, unison)
  - Smooth, zipper-free
  - Combines with vibrato LFO and analog drift

---

## Quick Reference Table

| CC# | Parameter | Range | Default | Saved |
|-----|-----------|-------|---------|-------|
| **16** | Analog Drift | 0-2 cents | 0 | ✓ |
| **17** | Env Release | 0-10s | 100ms | ✓ |
| **18** | LFO Rate | 0.1-20 Hz | 2 Hz | ✓ |
| **19** | Filter Drive | 1-6× | 1× | ✓ |
| **71** | LP Resonance | 0-1.0 | 0.7 | ✓ |
| **74** | LP Cutoff | 100-10k Hz | 1000 Hz | ✓ |
| **76** | HP Cutoff | 20-2k Hz | 20 Hz | ✓ |
| **77** | Thickness | 0-127 | 0 | ✓ |
| **82** | Env Attack | 0-5s | 10ms | ✓ |
| **83** | Env Decay | 0-10s | 50ms | ✓ |
| **85** | Env Sustain | 0-1.0 | 0.7 | ✓ |
| **93** | LFO Mode | 0-3 | 0 | ✓ |

---

## Preset Examples

### **Classic Analog Bass**
```
CC74 = 40   (Dark LP filter)
CC71 = 80   (High resonance)
CC77 = 60   (Sub + slight unison)
CC82 = 5    (Fast attack)
CC17 = 30   (Medium release)
CC19 = 50   (Slight drive)
```

### **Evolving Pad**
```
CC74 = 70   (Bright filter)
CC71 = 50   (Moderate resonance)
CC77 = 100  (Full thickness)
CC82 = 80   (Slow attack)
CC85 = 120  (High sustain)
CC17 = 100  (Long release)
CC18 = 20   (Slow LFO)
CC93 = 50   (Filter wobble)
```

### **Plucky Lead**
```
CC74 = 90   (Bright)
CC71 = 60   (Moderate resonance)
CC77 = 30   (Subtle thickness)
CC82 = 0    (Instant attack)
CC83 = 40   (Fast decay)
CC85 = 30   (Low sustain)
CC17 = 20   (Short release)
```

### **Distorted Wobble Bass**
```
CC74 = 50   (Medium cutoff)
CC71 = 100  (High resonance)
CC77 = 80   (Thick)
CC19 = 100  (Heavy drive)
CC18 = 90   (Fast LFO)
CC93 = 110  (S&H filter)
CC16 = 60   (Analog drift)
```

---

## Technical Notes

### Sample Rate & Latency
- **Sample Rate:** 44.1 kHz
- **Audio Latency:** 0.73 ms (16 samples × 2 buffers)
- **MIDI Latency:** Sub-millisecond (byte-by-byte processing)

### CPU Optimization
- Drift computed once per buffer (~2700 Hz update rate)
- Unison only active when CC77 > 57 (threshold ~32%)
- Thickness processing skipped entirely when CC77 = 0
- Pre-computed detune ratio and parameter smoothing

### Persistence
- All 12 controller values saved to NVS flash
- Auto-save 2 seconds after last change
- Settings restored on boot
- Flash-friendly (minimal writes)

---

## MIDI Implementation Chart

```
Function                 Transmitted  Recognized  Remarks
─────────────────────────────────────────────────────────
Basic Channel           -            1           Fixed
Mode                    -            Mode 3      Poly
Note Number             -            0-127
Velocity                -            Note On     0-127
Aftertouch              -            X           Not supported
Pitch Bend              -            O           ±2 semitones
Control Change          -            O           See table above
Program Change          -            X           Not supported
System Exclusive        -            X           Not supported
System Common           -            X           Not supported
System Real Time        -            X           Not supported
```

**Legend:** O = Supported, X = Not supported, - = N/A

---

## Version Info
- **Firmware:** ESP32 MIDI Synthesizer v1.0
- **Architecture:** Duophonic Paraphony
- **Platform:** ESP-IDF v5.4+
- **Hardware:** ESP32 + PCM5102 DAC
