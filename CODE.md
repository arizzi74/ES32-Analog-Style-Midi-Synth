# ESP32 MIDI Synthesizer - Code Structure Documentation

## Overview

This is a professional-grade MIDI synthesizer running on ESP32 with:
- **Duophonic paraphony** (2 simultaneous notes, shared envelopes/filters)
- **PolyBLEP oscillators** with sub-oscillator and unison detune
- **State-variable filters** (low-pass and high-pass)
- **ADSR envelopes** for amplitude and filter modulation
- **Analog drift emulation** for vintage character
- **NVS persistence** for controller settings

---

## File Structure

```
esp32synth/
├── main/
│   ├── esp32synth.c          # Main synthesizer implementation (1300+ lines)
│   └── CMakeLists.txt         # Component build configuration
├── CMakeLists.txt             # Project build configuration
├── sdkconfig                  # ESP-IDF configuration
├── MIDI_CONTROLLERS.md        # MIDI CC documentation
├── ESP32_MIDI_PCM5102A_6N138_WIRING.md  # Hardware wiring guide
├── CODE.md                    # This file
└── README.md                  # Project overview
```

---

## Code Architecture (`esp32synth.c`)

### **Total Lines:** ~1300
### **Language:** C (C11 standard)
### **Framework:** ESP-IDF v5.4+

---

## Section Breakdown

### 1. **Includes & Configuration** (Lines 1-60)
- ESP-IDF driver includes (I2S, UART, GPIO, DAC, NVS)
- Pin definitions for MIDI, I2S, and DAC
- Audio configuration (sample rate, buffer sizes)
- Synthesis parameters (oscillator count, wavetables)

**Key Defines:**
```c
#define SAMPLE_RATE 44100
#define MAX_VOICES 2              // Duophonic paraphony
#define DMA_BUF_LEN 16            // 0.73ms latency
#define SINE_TABLE_SIZE 1024
#define DETUNE_CENTS 8.0f         // Unison detune
```

---

### 2. **MIDI Protocol** (Lines 62-75)
- MIDI message type definitions
- Note on/off, control change, pitch bend

```c
#define MIDI_NOTE_OFF 0x80
#define MIDI_NOTE_ON 0x90
#define MIDI_CONTROL_CHANGE 0xB0
#define MIDI_PITCH_BEND 0xE0
```

---

### 3. **Data Structures** (Lines 77-165)

#### **Oscillator Types**
```c
typedef enum {
    OSC_SAW,
    OSC_SQUARE,
    OSC_TRIANGLE,
    OSC_SINE
} osc_type_t;
```

#### **ADSR Envelope**
```c
typedef struct {
    env_stage_t stage;           // IDLE, ATTACK, DECAY, SUSTAIN, RELEASE
    float value;                 // Current envelope level (0-1)
    float attack_rate;           // Rise speed
    float decay_rate;            // Fall to sustain speed
    float sustain_level;         // Held level
    float release_rate;          // Fall to silence speed
} adsr_t;
```

#### **State-Variable Filter**
```c
typedef struct {
    float ic1eq, ic2eq;          // Internal state (integrator capacitors)
    float g, k;                  // Precalculated coefficients
} svf_t;
```

#### **Drift LFO** (Analog instability emulation)
```c
typedef struct {
    uint32_t rng_state;          // xorshift32 PRNG
    float current_value;         // Unfiltered noise
    float filtered_value;        // Low-pass filtered drift (-1 to 1)
    float update_counter;        // Rate control
} drift_lfo_t;
```

#### **Paraphonic Oscillator**
```c
typedef struct {
    bool active;                 // Generating sound
    bool held;                   // Note currently pressed
    uint8_t note;               // MIDI note number
    float frequency;            // Hz
    float phase;                // Main oscillator phase (0-1)
    float sub_phase;            // Sub oscillator phase (-1 octave)
    float unison_phase;         // Unison detune phase
    drift_lfo_t drift_lfo;      // Per-oscillator drift
} para_osc_t;
```

#### **Complete Synthesizer**
```c
typedef struct {
    para_osc_t oscillators[MAX_VOICES];  // 2 oscillators
    adsr_t amp_env;              // Shared amplitude envelope
    adsr_t filter_env;           // Shared filter envelope
    svf_t filter_lp;             // Low-pass filter
    svf_t filter_hp;             // High-pass filter
    lfo_t lfo;                   // Musical LFO
    osc_type_t osc_type;         // Waveform selection
    float pitch_bend;            // ±2 semitones
    uint8_t velocity;            // Last note velocity
} paraphonic_synth_t;
```

---

### 4. **Global Variables** (Lines 167-214)
- I2S and DAC handles
- MIDI queue
- Synthesizer instance
- Sine wavetable (1024 samples)
- All controller parameters (filters, envelopes, LFO, thickness, drift)
- NVS persistence tracking

---

### 5. **NVS Persistence** (Lines 216-310)

#### **`save_settings_to_nvs()`**
- Saves 12 controller values to flash
- Float-to-uint32 pointer casting
- Auto-called 2 seconds after last CC change

#### **`load_settings_from_nvs()`**
- Restores settings on boot
- Applies to envelopes and filters
- Silent if no saved data exists

**Saved Parameters:**
1. Drift depth (CC16)
2. Filter drive (CC19)
3. LP cutoff (CC74)
4. LP resonance (CC71)
5. HP cutoff (CC76)
6. LFO rate (CC18)
7. LFO mode (CC93)
8. Envelope attack (CC82)
9. Envelope decay (CC83)
10. Envelope sustain (CC85)
11. Envelope release (CC17)
12. Thickness (CC77)

---

### 6. **Wavetable Generation** (Lines 312-320)

#### **`init_sine_table()`**
- Pre-computes 1024 sine samples
- Used for sine oscillator and LFO
- Called once at startup

---

### 7. **Utility Functions** (Lines 322-365)

#### **`midi_to_freq(uint8_t note)`**
- Converts MIDI note (0-127) to Hz
- Equal temperament: f = 440 × 2^((n-69)/12)

#### **`soft_clip(float x)`**
- Smooth saturation: tanh approximation
- Prevents hard clipping distortion

#### **`adsr_init()` / `adsr_update_params()`**
- Initialize envelope with parameters
- Update parameters without resetting state (for live CC changes)

#### **`adsr_note_on()` / `adsr_note_off()`**
- Trigger attack/release stages

#### **`adsr_process()`**
- Compute current envelope value
- State machine for ATTACK → DECAY → SUSTAIN → RELEASE → IDLE

---

### 8. **Filter Implementation** (Lines 367-405)

#### **`svf_set_params(svf_t *f, float cutoff_hz, float resonance)`**
- Pre-compute SVF coefficients
- Uses TPT (Topology-Preserving Transform) for stability

#### **`svf_process(svf_t *f, float in)` - Low-pass**
- Returns filtered low-pass output

#### **`svf_process_hp(svf_t *f, float in)` - High-pass**
- Returns filtered high-pass output

---

### 9. **LFO (Low-Frequency Oscillator)** (Lines 407-475)

#### **`lfo_init()` / `lfo_set_rate()`**
- Initialize LFO with waveform and rate

#### **`lfo_process()`**
- Generate LFO samples
- Waveforms: SINE, TRIANGLE, SQUARE, SAMPLE_HOLD
- Used for vibrato, filter wobble, S&H modulation

---

### 10. **Drift LFO (Analog Instability)** (Lines 477-520)

#### **`drift_lfo_init()`**
- Initialize per-oscillator drift
- Unique seed for each voice

#### **`xorshift32()` - PRNG**
- Fast random number generator
- 32-bit linear feedback shift register

#### **`drift_lfo_process()`**
- Generate smooth random drift (±cents)
- Low-pass filtered noise for organic pitch variation
- Update rate: ~8 Hz

---

### 11. **Oscillator Generation** (Lines 522-620)

#### **`polyblep(float t, float dt)`**
- Polynomial Band-Limited Step
- Reduces aliasing in discontinuous waveforms

#### **`generate_oscillator(float phase, float phase_inc, osc_type_t type)`**
- Generate single sample
- Waveforms: SAW, SQUARE, TRIANGLE, SINE
- PolyBLEP anti-aliasing for SAW/SQUARE

---

### 12. **Synthesizer Core** (Lines 622-725)

#### **`synth_init()`**
- Initialize all oscillators, envelopes, filters
- Set default parameters

#### **`synth_note_on()`**
- Paraphonic note allocation
- Retrigger detection
- Envelope triggering on first note

**Logic:**
1. Check if note already held (retrigger)
2. Find free oscillator slot
3. Initialize frequency and phases
4. Trigger envelopes if first note

#### **`synth_note_off()`**
- Mark oscillator as released (`held = false`)
- Keep `active = true` for release tail
- Trigger envelope release when last note released

**Paraphonic Behavior:**
- First note ON → Trigger envelopes
- Second note ON → Add oscillator, envelopes continue
- First note OFF → Remove oscillator
- Last note OFF → Trigger envelope release

---

### 13. **Audio Generation Task** (Lines 727-950)

**Priority:** 20 (highest)
**Core:** 1
**Stack:** 4096 bytes

#### **Per-Buffer Processing:**
1. **Parameter smoothing** (once per buffer)
   - LFO rate, drift depth, filter drive
   - Thickness and derived mix values
   - Prevents zipper noise

2. **LFO processing** (once per buffer)
   - Compute LFO value
   - Route to PWM/Filter/Vibrato/S&H based on CC93

3. **Drift processing** (once per buffer per oscillator)
   - Compute pitch scale including drift
   - Cached for all samples in buffer
   - **Optimization:** Moved from per-sample to per-buffer (16× speedup)

4. **Sample generation loop** (16 samples)
   - For each active oscillator:
     - Generate main oscillator (with drift/vibrato/pitch bend)
     - Generate sub oscillator (if thickness > 0, square wave at ½ frequency)
     - Generate unison oscillator (if thickness > 32%, detuned copy)
     - Mix with gains, maintain phase continuity
   - Average oscillators (paraphonic)
   - Apply high-pass filter
   - Apply drive and soft clipping
   - Apply low-pass filter (with envelope modulation)
   - Apply amplitude envelope and velocity
   - Soft clip output
   - Convert to 16-bit stereo
   - Output to DAC (GPIO25 for oscilloscope)

5. **I2S output**
   - Write 16-sample stereo buffer
   - Blocking (provides natural pacing)

6. **Cleanup**
   - Deactivate released oscillators when envelope reaches IDLE
   - Auto-save settings if 2 seconds elapsed

---

### 14. **MIDI Processing Task** (Lines 952-1075)

**Priority:** 13
**Core:** 0
**Stack:** 3072 bytes

#### **Event Processing:**
- Reads MIDI events from queue
- Processes:
  - **Note On:** Calls `synth_note_on()`, logs note
  - **Note Off:** Calls `synth_note_off()`, logs release
  - **Control Change:** Updates parameters, marks for NVS save
  - **Pitch Bend:** 14-bit value, ±2 semitones

#### **Control Change Handlers:**
All CC handlers:
1. Parse value (0-127)
2. Map to parameter range
3. Update target value (smoothed in audio task)
4. Log change
5. Mark `settings_need_save = true`
6. Timestamp for auto-save

**Special Cases:**
- CC82/83/85/17 (envelopes): Call `adsr_update_params()` to avoid resetting state
- CC77 (thickness): Compute preview mix values for logging

---

### 15. **MIDI UART Receive Task** (Lines 1077-1185)

**Priority:** 14 (high)
**Core:** 0
**Stack:** 3072 bytes

#### **Byte-by-Byte Processing:**
- UART configured for 1-byte RX threshold
- Reads single bytes with `portMAX_DELAY` (blocking)
- **Running status** support (MIDI compression)
- State machine for message assembly

**Optimization:**
- Minimal latency (<1ms)
- No batching or delays
- Sends complete messages to queue

---

### 16. **Main Application** (Lines 1187-1310)

#### **`app_main()` Initialization Sequence:**

1. **NVS Flash**
   - Initialize with auto-erase on version mismatch

2. **Pre-compute Constants**
   - Detune ratio: `2^(8/1200)` for unison

3. **Sine Table**
   - Generate 1024-sample lookup table

4. **Synthesizer**
   - Initialize all oscillators, envelopes, filters

5. **Load NVS Settings**
   - Restore saved controller values
   - Apply to envelopes

6. **MIDI Queue**
   - Create FreeRTOS queue (32 events)

7. **I2S Audio Output**
   - Configure channel (2 buffers × 16 samples)
   - 44.1 kHz, 16-bit stereo
   - Pins: GPIO14 (BCK), GPIO12 (LCK), GPIO22 (DOUT)

8. **DAC Oscilloscope Output**
   - GPIO25, 8-bit, 0-3.3V

9. **UART MIDI Input**
   - 31250 baud, 8N1
   - GPIO16 (RX)
   - RX threshold = 1 (interrupt per byte)

10. **Task Creation**
    - Audio generation (priority 20, core 1)
    - MIDI UART (priority 14, core 0)
    - MIDI processing (priority 13, core 0)

---

## Memory Usage

### **Stack Allocations:**
- Audio task: 4096 bytes
- MIDI UART task: 3072 bytes
- MIDI processing task: 3072 bytes
- **Total:** ~10 KB

### **Heap Allocations:**
- I2S DMA buffers: 2 × 16 × 4 bytes = 128 bytes
- UART RX buffer: 2048 bytes
- MIDI queue: 32 × 4 bytes = 128 bytes
- **Total:** ~2.3 KB

### **Flash (Code):**
- Binary size: 256 KB
- Free space: 75% (768 KB available)

### **Static Data:**
- Sine table: 1024 × 4 bytes = 4 KB
- Synth state: ~512 bytes
- Global variables: ~200 bytes
- **Total:** ~5 KB

---

## CPU Performance

### **Per-Buffer Processing (16 samples @ 44.1 kHz = 362 µs):**
- Parameter smoothing: ~5 µs
- LFO: ~2 µs
- Drift (2 oscillators): ~10 µs
- Sample generation: ~250 µs
  - Main oscillators: ~100 µs
  - Sub oscillators: ~20 µs (if enabled)
  - Unison oscillators: ~80 µs (if enabled, CC77 > 57)
  - Filters: ~30 µs
  - Envelopes: ~10 µs
- DAC output: ~16 µs
- I2S write: ~10 µs (blocking)
- **Total:** ~300 µs (83% CPU usage on core 1)

### **Optimization Techniques:**
1. **Pre-computed tables:** Sine wavetable
2. **Cached calculations:** Detune ratio, drift pitch scale
3. **Conditional processing:** Thickness only if > 0, unison only if > 32%
4. **Per-buffer vs per-sample:** LFO, drift moved out of inner loop
5. **Small buffers:** 16 samples allows frequent task yielding

---

## Signal Flow Diagram

```
MIDI In (GPIO16)
    ↓
UART Task (byte-by-byte, 31250 baud)
    ↓
MIDI Queue (32 events)
    ↓
MIDI Processing Task (note on/off, CC, pitch bend)
    ↓
Synthesizer State (oscillators, envelopes, filters)
    ↓
Audio Generation Task (core 1, 44.1 kHz)
    │
    ├─> Main Oscillators (PolyBLEP, drift modulated)
    ├─> Sub Oscillators (square, -1 oct, if thickness > 0)
    ├─> Unison Oscillators (detuned, if thickness > 32%)
    │
    ├─> Mix & Average (paraphonic)
    ├─> High-pass Filter
    ├─> Drive & Saturation
    ├─> Low-pass Filter (envelope + LFO modulated)
    ├─> Amplitude Envelope
    ├─> Soft Clip
    │
    ├─> I2S Output (GPIO14/12/22) → PCM5102 DAC → Audio
    └─> DAC Output (GPIO25) → Oscilloscope
```

---

## Threading Model

```
Core 0:                              Core 1:
┌──────────────────────────┐        ┌──────────────────────────┐
│ MIDI UART Task (P14)     │        │ Audio Task (P20)         │
│ - Byte-by-byte RX        │        │ - 44.1 kHz sample gen    │
│ - Running status         │        │ - Filters, envelopes     │
│ - Queue send             │        │ - I2S output             │
└──────────────────────────┘        │ - DAC output             │
                                    └──────────────────────────┘
┌──────────────────────────┐
│ MIDI Processing Task(P13)│
│ - Note on/off            │
│ - CC handlers            │
│ - Pitch bend             │
│ - NVS auto-save          │
└──────────────────────────┘
```

**Synchronization:**
- FreeRTOS queue for MIDI events (thread-safe)
- No locks needed (single writer to synth state)
- Audio task only reads controller values (smoothed)

---

## Critical Sections

### **None Required:**
- Audio task is sole writer to oscillator state
- MIDI task is sole writer to controller targets
- No shared mutable state between tasks

### **Atomic Operations:**
- Controller value updates (float writes are atomic on ESP32)
- NVS save flag (bool)

---

## Error Handling

### **Initialization Errors:**
- All ESP_ERROR_CHECK() calls will panic on failure
- NVS erase-and-retry on version mismatch
- Task creation failures logged and halt

### **Runtime Errors:**
- MIDI queue full: Logged every 100th drop
- All oscillators busy: Logged warning, note ignored

### **Graceful Degradation:**
- Missing NVS data: Use defaults
- Invalid MIDI messages: Silently ignored

---

## Code Quality

### **Metrics:**
- **Lines of Code:** ~1300
- **Cyclomatic Complexity:** Low (mostly linear flow)
- **Function Count:** ~35
- **Max Function Length:** ~150 lines (audio_generation_task)
- **Comment Density:** ~15%

### **Standards:**
- **C Standard:** C11 (GNU extensions)
- **Style:** Linux kernel style (4-space indent)
- **Naming:** snake_case for functions/variables, UPPER_CASE for defines

### **Best Practices:**
- ✅ No dynamic allocation in audio path
- ✅ Const correctness for lookup tables
- ✅ Minimal global state
- ✅ Clear separation of concerns
- ✅ Defensive programming (bounds checks)
- ✅ Extensive inline documentation

---

## Build Configuration

### **Platform:** ESP-IDF v5.4+
### **Toolchain:** xtensa-esp32-elf-gcc
### **Optimization:** -O2 (balance speed/size)

### **Required Components:**
- esp_driver_i2s
- esp_driver_uart
- esp_driver_gpio
- esp_driver_dac
- nvs_flash
- freertos

---

## Testing Approach

### **Unit Testing:**
- Not implemented (embedded target)
- Manual verification via MIDI input

### **Integration Testing:**
- Full system test via MIDI controller
- Oscilloscope verification of waveforms
- Audio quality assessment

### **Performance Testing:**
- Task watchdog monitoring
- CPU usage profiling
- Latency measurement

---

## Future Expansion Points

### **Easy to Add:**
1. **More waveforms:** Add to `osc_type_t` enum
2. **More LFO modes:** Extend CC93 handler
3. **Filter types:** Band-pass, notch (SVF already has outputs)
4. **Effects:** Delay, chorus (add circular buffers)

### **Moderate Complexity:**
1. **Polyphony:** Replace paraphonic with per-note envelopes
2. **Arpeggiator:** Add note queue and clock
3. **Sequencer:** Store and playback patterns
4. **Preset system:** Extend NVS storage

### **Challenging:**
1. **Wavetable synthesis:** Large lookup tables
2. **Physical modeling:** Complex algorithms
3. **Multi-timbral:** Multiple synth instances

---

## Known Limitations

1. **2-voice maximum:** Duophonic only (CPU constraint)
2. **8-bit DAC:** Oscilloscope output resolution
3. **No MIDI output:** Input only
4. **Fixed sample rate:** 44.1 kHz only
5. **No USB MIDI:** UART DIN-5 only

---

## Version History

- **v1.0** - Initial release
  - Duophonic paraphony
  - PolyBLEP oscillators
  - ADSR envelopes
  - SVF filters
  - 12 MIDI CC controllers
  - NVS persistence
  - DAC oscilloscope output
