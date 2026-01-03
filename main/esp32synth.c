/*
 * ESP32 MIDI Synthesizer
 * Professional monophonic synth with PolyBLEP oscillators, ADSR, and SVF filter
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/dac_oneshot.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "ESP32SYNTH";

// ============================================================================
// CONFIGURATION
// ============================================================================

// MIDI UART configuration
#define MIDI_UART_NUM UART_NUM_1
#define MIDI_RX_PIN GPIO_NUM_16
#define MIDI_BAUD_RATE 31250
#define UART_BUF_SIZE 1024

// I2S configuration for PCM5102
// Remapped to free GPIO25 for DAC output
#define I2S_BCLK_PIN GPIO_NUM_14
#define I2S_LCK_PIN GPIO_NUM_12
#define I2S_DOUT_PIN GPIO_NUM_22
#define SAMPLE_RATE 44100
#define I2S_NUM I2S_NUM_0

// DAC configuration for oscilloscope output
#define DAC_CHANNEL DAC_CHAN_0  // GPIO25 (DAC Channel 0)
#define DAC_OUTPUT_PIN GPIO_NUM_25

// LATENCY TUNING:
// Total latency = (DMA_BUF_COUNT × DMA_BUF_LEN) / SAMPLE_RATE
// Current: (2 × 16) / 44100 = 0.73ms (ultra-low latency for live performance)
//
// To adjust:
// - Smaller buffers = lower latency + more frequent yielding (prevents watchdog)
// - Larger buffers = higher latency but less CPU overhead
// Reduced to 16 samples to handle phase-continuous sub+unison without overruns
#define DMA_BUF_COUNT 2
#define DMA_BUF_LEN 16

// Audio synthesis
#define PI 3.14159265358979323846f
#define TWO_PI (2.0f * PI)
#define SINE_TABLE_SIZE 1024
#define MAX_VOICES 2  // Paraphony: 2 simultaneous notes

// MIDI message types
#define MIDI_NOTE_OFF 0x80
#define MIDI_NOTE_ON 0x90
#define MIDI_CONTROL_CHANGE 0xB0
#define MIDI_PITCH_BEND 0xE0

// Oscillator types
typedef enum {
    OSC_SINE = 0,
    OSC_SAW,
    OSC_PULSE,
    OSC_COUNT
} osc_type_t;

const char* osc_names[] = {"Sine", "Saw", "Pulse"};

// ADSR envelope stages
typedef enum {
    ENV_IDLE = 0,
    ENV_ATTACK,
    ENV_DECAY,
    ENV_SUSTAIN,
    ENV_RELEASE
} env_stage_t;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// ADSR Envelope
typedef struct {
    env_stage_t stage;
    float value;           // Current envelope value [0, 1]
    float attack_rate;     // Rate per sample
    float decay_rate;
    float sustain_level;   // [0, 1]
    float release_rate;
} adsr_t;

// State-Variable Filter (SVF)
typedef struct {
    float lp;      // Low-pass output
    float bp;      // Band-pass output
    float hp;      // High-pass output
    float f;       // Frequency coefficient
    float q;       // Resonance (1/Q)
} svf_t;

// LFO waveform types
typedef enum {
    LFO_TRIANGLE = 0,
    LFO_SINE,
    LFO_SQUARE,
    LFO_SAMPLE_HOLD
} lfo_waveform_t;

// LFO (Low Frequency Oscillator)
typedef struct {
    float phase;           // Current phase [0, 1)
    float rate_hz;         // Frequency in Hz
    lfo_waveform_t waveform;
    float value;           // Current LFO output [-1, 1]
    float sh_value;        // Sample & hold value
    float sh_timer;        // S&H clock timer
} lfo_t;

// Drift LFO (smooth random per oscillator for analog feel)
typedef struct {
    uint32_t rng_state;    // xorshift32 PRNG state
    float current_value;   // Current drift value (unfiltered noise)
    float filtered_value;  // Low-pass filtered drift [-1, 1]
    float update_counter;  // Counter for update rate
} drift_lfo_t;

// Paraphonic oscillator (individual note/frequency)
typedef struct {
    bool active;      // Oscillator is generating sound
    bool held;        // Note is currently pressed (not released)
    uint8_t note;
    float frequency;
    float phase;      // Main oscillator phase
    float sub_phase;  // Sub oscillator phase (-1 octave, square wave)
    float unison_phase; // Unison detune oscillator phase
    drift_lfo_t drift_lfo; // Per-oscillator smooth random drift
} para_osc_t;

// Paraphonic synth (shared envelopes/filter)
typedef struct {
    para_osc_t oscillators[MAX_VOICES];  // Up to 8 oscillators

    adsr_t amp_env;        // Shared amplitude envelope
    adsr_t filter_env;     // Shared filter envelope
    svf_t filter_lp;       // Shared low-pass filter
    svf_t filter_hp;       // Shared high-pass filter
    lfo_t lfo;             // Shared musical LFO

    osc_type_t osc_type;   // Shared oscillator type
    float pitch_bend;      // Shared pitch bend
    uint8_t velocity;      // Last note velocity
    int active_count;      // Number of active oscillators
} paraphonic_synth_t;

// MIDI event for queue (packed to avoid alignment issues)
typedef struct __attribute__((packed)) {
    uint8_t status;
    uint8_t data1;
    uint8_t data2;
    uint8_t padding;  // Align to 4 bytes for ESP32
} midi_event_t;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

static i2s_chan_handle_t tx_handle = NULL;
static dac_oneshot_handle_t dac_handle = NULL;
static QueueHandle_t midi_queue = NULL;
static paraphonic_synth_t synth = {0};

// Precomputed sine table
static float sine_table[SINE_TABLE_SIZE];

// Synth parameters
static float filter_cutoff = 1000.0f;     // LP cutoff Hz
static float filter_resonance = 0.7f;     // LP resonance 0-1
static float filter_env_amount = 0.5f;    // Filter envelope amount 0-1
static float hp_cutoff = 20.0f;           // HP cutoff Hz (CC76)

// LFO parameters (CC18 = rate, CC93 = mode)
static float lfo_rate = 2.0f;              // LFO rate Hz (CC18: 1-120 Hz)
static float lfo_rate_target = 2.0f;       // Smoothed target
static uint8_t lfo_mode = 0;               // CC93 mode: 0-31=PWM, 32-63=Filter, 64-95=Vibrato, 96-127=S&H
static float lfo_depth = 0.5f;             // LFO depth (0-1)

// "Super Analog" features (CC16 = drift, CC19 = drive)
static float drift_depth = 0.0f;           // CC16: Analog drift depth (0-1, maps to 0.00-2.00 cents)
static float drift_depth_target = 0.0f;    // Smoothed target
static float filter_drive = 1.0f;          // CC19: Pre-filter drive 1.0-6.0
static float filter_drive_target = 1.0f;   // Smoothed target

// ADSR envelope parameters (CC82=A, CC83=D, CC85=S, CC17=R)
// Shared by both amp and filter envelopes (paraphonic)
static float env_attack_ms = 10.0f;        // CC82: 0-5000 ms
static float env_decay_ms = 50.0f;         // CC83: 0-10000 ms
static float env_sustain = 0.7f;           // CC85: 0-1
static float env_release_ms = 100.0f;      // CC17: 0-10000 ms

// NVS persistence
static int64_t last_cc_change_time = 0;    // Timestamp of last CC change
static bool settings_need_save = false;    // Flag to trigger save

// Thickness control (CC77: sub + unison mix)
#define DETUNE_CENTS 8.0f                  // Unison detune amount in cents (3-12 range)
static float detune_ratio = 0.0f;          // Pre-computed: 2^(DETUNE_CENTS/1200), set in app_main
static float thickness_target = 0.0f;      // CC77 mapped to 0.0-1.0
static float thickness_current = 0.0f;     // Smoothed thickness
static float unison_mix = 0.0f;            // Derived unison blend (0-0.5)
static float sub_mix = 0.0f;               // Derived sub blend (0-0.35)

// ============================================================================
// NVS SETTINGS PERSISTENCE
// ============================================================================

void save_settings_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("synth", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return;
    }

    // Save all controller values
    nvs_set_u32(nvs_handle, "drift_depth", *(uint32_t*)&drift_depth_target);
    nvs_set_u32(nvs_handle, "filter_drive", *(uint32_t*)&filter_drive_target);
    nvs_set_u32(nvs_handle, "filter_cut", *(uint32_t*)&filter_cutoff);
    nvs_set_u32(nvs_handle, "filter_res", *(uint32_t*)&filter_resonance);
    nvs_set_u32(nvs_handle, "hp_cutoff", *(uint32_t*)&hp_cutoff);
    nvs_set_u32(nvs_handle, "lfo_rate", *(uint32_t*)&lfo_rate_target);
    nvs_set_u8(nvs_handle, "lfo_mode", lfo_mode);
    nvs_set_u32(nvs_handle, "env_attack", *(uint32_t*)&env_attack_ms);
    nvs_set_u32(nvs_handle, "env_decay", *(uint32_t*)&env_decay_ms);
    nvs_set_u32(nvs_handle, "env_sustain", *(uint32_t*)&env_sustain);
    nvs_set_u32(nvs_handle, "env_release", *(uint32_t*)&env_release_ms);
    nvs_set_u32(nvs_handle, "thickness", *(uint32_t*)&thickness_target);

    // Commit and close
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Settings saved to NVS");
        settings_need_save = false;
    } else {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    }
}

void load_settings_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("synth", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved settings found, using defaults");
        return;
    }

    uint32_t temp_u32;
    uint8_t temp_u8;

    // Load all controller values
    if (nvs_get_u32(nvs_handle, "drift_depth", &temp_u32) == ESP_OK) {
        drift_depth_target = *(float*)&temp_u32;
        drift_depth = drift_depth_target;
    }
    if (nvs_get_u32(nvs_handle, "filter_drive", &temp_u32) == ESP_OK) {
        filter_drive_target = *(float*)&temp_u32;
        filter_drive = filter_drive_target;
    }
    if (nvs_get_u32(nvs_handle, "filter_cut", &temp_u32) == ESP_OK) {
        filter_cutoff = *(float*)&temp_u32;
    }
    if (nvs_get_u32(nvs_handle, "filter_res", &temp_u32) == ESP_OK) {
        filter_resonance = *(float*)&temp_u32;
    }
    if (nvs_get_u32(nvs_handle, "hp_cutoff", &temp_u32) == ESP_OK) {
        hp_cutoff = *(float*)&temp_u32;
    }
    if (nvs_get_u32(nvs_handle, "lfo_rate", &temp_u32) == ESP_OK) {
        lfo_rate_target = *(float*)&temp_u32;
        lfo_rate = lfo_rate_target;
    }
    if (nvs_get_u8(nvs_handle, "lfo_mode", &temp_u8) == ESP_OK) {
        lfo_mode = temp_u8;
    }
    if (nvs_get_u32(nvs_handle, "env_attack", &temp_u32) == ESP_OK) {
        env_attack_ms = *(float*)&temp_u32;
    }
    if (nvs_get_u32(nvs_handle, "env_decay", &temp_u32) == ESP_OK) {
        env_decay_ms = *(float*)&temp_u32;
    }
    if (nvs_get_u32(nvs_handle, "env_sustain", &temp_u32) == ESP_OK) {
        env_sustain = *(float*)&temp_u32;
    }
    if (nvs_get_u32(nvs_handle, "env_release", &temp_u32) == ESP_OK) {
        env_release_ms = *(float*)&temp_u32;
    }
    if (nvs_get_u32(nvs_handle, "thickness", &temp_u32) == ESP_OK) {
        thickness_target = *(float*)&temp_u32;
        thickness_current = thickness_target;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Settings loaded from NVS");
}

// ============================================================================
// SINE TABLE GENERATION
// ============================================================================

void init_sine_table(void) {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        sine_table[i] = sinf(TWO_PI * i / SINE_TABLE_SIZE);
    }
}

float lookup_sine(float phase) {
    // phase is [0, 1)
    float index = phase * SINE_TABLE_SIZE;
    int i0 = (int)index;
    int i1 = (i0 + 1) % SINE_TABLE_SIZE;
    float frac = index - i0;

    // Linear interpolation
    return sine_table[i0] + frac * (sine_table[i1] - sine_table[i0]);
}

// ============================================================================
// POLYBLEP ANTI-ALIASING
// ============================================================================

// PolyBLEP residual for band-limited step
float poly_blep(float t, float dt) {
    // t is phase position, dt is phase increment
    if (t < dt) {
        t /= dt;
        return t + t - t * t - 1.0f;
    } else if (t > 1.0f - dt) {
        t = (t - 1.0f) / dt;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}

// Soft saturation using tanh approximation (prevents harsh clipping)
float soft_clip(float x) {
    // Fast tanh approximation: x / (1 + |x|)
    // Smoother than hard clipping, prevents digital harshness
    if (x > 3.0f) return 1.0f;
    if (x < -3.0f) return -1.0f;
    return x / (1.0f + fabsf(x) * 0.3f);
}

// ============================================================================
// ANALOG DRIFT LFO (SMOOTH RANDOM)
// ============================================================================

// xorshift32 PRNG (deterministic, no libc rand())
uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

// Initialize drift LFO with unique seed per oscillator
void drift_lfo_init(drift_lfo_t *drift, uint8_t osc_index) {
    // Unique seed per oscillator (avoid all zeros)
    drift->rng_state = 0x12345678 + (osc_index * 0x9ABCDEF);
    drift->current_value = 0.0f;
    drift->filtered_value = 0.0f;
    drift->update_counter = 0.0f;
}

// Process drift LFO - generates smooth random drift
// Returns drift in cents (±depth_cents)
float drift_lfo_process(drift_lfo_t *drift, float depth_cents) {
    if (depth_cents <= 0.0f) {
        return 0.0f;  // No drift if depth is zero
    }

    // Update rate: ~0.1-0.2 Hz (very slow, feels like analog VCO drift)
    // Update every ~5000-10000 samples at 44.1kHz = 0.11-0.22 seconds
    const float UPDATE_RATE = 0.0001f;  // Adjust for drift speed

    drift->update_counter += UPDATE_RATE;
    if (drift->update_counter >= 1.0f) {
        drift->update_counter = 0.0f;

        // Generate new random value [-1, 1]
        uint32_t rnd = xorshift32(&drift->rng_state);
        drift->current_value = ((float)rnd / (float)UINT32_MAX) * 2.0f - 1.0f;
    }

    // Low-pass filter the noise (smooth it out)
    // Very slow filter cutoff (~0.2 Hz equivalent)
    const float FILTER_COEFF = 0.9995f;  // Higher = smoother, slower changes
    drift->filtered_value = FILTER_COEFF * drift->filtered_value +
                           (1.0f - FILTER_COEFF) * drift->current_value;

    // Scale to requested depth in cents (±depth_cents)
    return drift->filtered_value * depth_cents;
}

// ============================================================================
// OSCILLATORS WITH POLYBLEP
// ============================================================================

float generate_oscillator(float phase, float phase_inc, osc_type_t type) {
    switch (type) {
        case OSC_SINE:
            return lookup_sine(phase);

        case OSC_SAW: {
            // Naive sawtooth
            float naive = 2.0f * phase - 1.0f;
            // Apply PolyBLEP
            naive -= poly_blep(phase, phase_inc);
            return naive;
        }

        case OSC_PULSE: {
            // Pulse wave with 50% duty cycle (square wave)
            float naive = (phase < 0.5f) ? 1.0f : -1.0f;
            // Apply PolyBLEP at rising edge (phase = 0)
            naive += poly_blep(phase, phase_inc);
            // Apply PolyBLEP at falling edge (phase = 0.5)
            naive -= poly_blep(fmodf(phase + 0.5f, 1.0f), phase_inc);
            return naive;
        }

        default:
            return 0.0f;
    }
}

// ============================================================================
// ADSR ENVELOPE
// ============================================================================

void adsr_init(adsr_t *env, float attack_ms, float decay_ms, float sustain_level, float release_ms) {
    env->stage = ENV_IDLE;
    env->value = 0.0f;

    // Convert times to rates (per sample)
    env->attack_rate = 1.0f / (attack_ms * 0.001f * SAMPLE_RATE);
    env->decay_rate = 1.0f / (decay_ms * 0.001f * SAMPLE_RATE);
    env->sustain_level = sustain_level;
    env->release_rate = 1.0f / (release_ms * 0.001f * SAMPLE_RATE);
}

// Update envelope parameters without resetting state (for live CC changes)
void adsr_update_params(adsr_t *env, float attack_ms, float decay_ms, float sustain_level, float release_ms) {
    // Update rates without resetting stage or value
    env->attack_rate = 1.0f / (attack_ms * 0.001f * SAMPLE_RATE);
    env->decay_rate = 1.0f / (decay_ms * 0.001f * SAMPLE_RATE);
    env->sustain_level = sustain_level;
    env->release_rate = 1.0f / (release_ms * 0.001f * SAMPLE_RATE);
}

void adsr_note_on(adsr_t *env) {
    env->stage = ENV_ATTACK;
}

void adsr_note_off(adsr_t *env) {
    env->stage = ENV_RELEASE;
}

float adsr_process(adsr_t *env) {
    switch (env->stage) {
        case ENV_IDLE:
            env->value = 0.0f;
            break;

        case ENV_ATTACK:
            env->value += env->attack_rate;
            if (env->value >= 1.0f) {
                env->value = 1.0f;
                env->stage = ENV_DECAY;
            }
            break;

        case ENV_DECAY:
            env->value -= env->decay_rate * (env->value - env->sustain_level);
            if (env->value <= env->sustain_level) {
                env->value = env->sustain_level;
                env->stage = ENV_SUSTAIN;
            }
            break;

        case ENV_SUSTAIN:
            env->value = env->sustain_level;
            break;

        case ENV_RELEASE:
            env->value -= env->release_rate * env->value;
            // More aggressive threshold to prevent stuck voices
            if (env->value <= 0.01f) {
                env->value = 0.0f;
                env->stage = ENV_IDLE;
            }
            break;
    }

    return env->value;
}

// ============================================================================
// STATE-VARIABLE FILTER (SVF)
// ============================================================================

void svf_init(svf_t *filter) {
    filter->lp = 0.0f;
    filter->bp = 0.0f;
    filter->hp = 0.0f;
    filter->f = 0.5f;
    filter->q = 0.5f;
}

void svf_set_params(svf_t *filter, float cutoff_hz, float resonance) {
    // Cutoff frequency coefficient
    // f = 2 * sin(π * cutoff / sample_rate)
    float cutoff_clamped = fminf(cutoff_hz, SAMPLE_RATE * 0.45f); // Nyquist limit
    filter->f = 2.0f * sinf(PI * cutoff_clamped / SAMPLE_RATE);

    // Resonance (inverse of Q)
    // Higher resonance = lower q value
    filter->q = 1.0f - resonance * 0.99f; // Prevent instability
}

float svf_process(svf_t *filter, float input) {
    // State-variable filter topology
    filter->lp += filter->f * filter->bp;
    filter->hp = input - filter->lp - filter->q * filter->bp;
    filter->bp += filter->f * filter->hp;

    // Safety: Reset filter if NaN/Inf detected (prevents stuck voices)
    if (!isfinite(filter->lp) || !isfinite(filter->bp) || !isfinite(filter->hp)) {
        filter->lp = filter->bp = filter->hp = 0.0f;
        return 0.0f;
    }

    // Return low-pass output
    return filter->lp;
}

float svf_process_hp(svf_t *filter, float input) {
    // State-variable filter topology (same as LP, but return HP)
    filter->lp += filter->f * filter->bp;
    filter->hp = input - filter->lp - filter->q * filter->bp;
    filter->bp += filter->f * filter->hp;

    // Safety: Reset filter if NaN/Inf detected
    if (!isfinite(filter->lp) || !isfinite(filter->bp) || !isfinite(filter->hp)) {
        filter->lp = filter->bp = filter->hp = 0.0f;
        return 0.0f;
    }

    // Return high-pass output
    return filter->hp;
}

// ============================================================================
// LFO (LOW FREQUENCY OSCILLATOR)
// ============================================================================

void lfo_init(lfo_t *lfo, float rate_hz) {
    lfo->phase = 0.0f;
    lfo->rate_hz = rate_hz;
    lfo->waveform = LFO_TRIANGLE;
    lfo->value = 0.0f;
    lfo->sh_value = 0.0f;
    lfo->sh_timer = 0.0f;
}

float lfo_process(lfo_t *lfo) {
    // Update phase
    float phase_inc = lfo->rate_hz / SAMPLE_RATE;
    lfo->phase += phase_inc;
    if (lfo->phase >= 1.0f) lfo->phase -= 1.0f;

    // Generate waveform
    switch (lfo->waveform) {
        case LFO_TRIANGLE:
            // Triangle: -1 to +1
            lfo->value = (lfo->phase < 0.5f)
                ? (lfo->phase * 4.0f - 1.0f)      // Rising
                : (3.0f - lfo->phase * 4.0f);    // Falling
            break;

        case LFO_SINE:
            // Sine wave using lookup table
            lfo->value = lookup_sine(lfo->phase);
            break;

        case LFO_SQUARE:
            // Square wave
            lfo->value = (lfo->phase < 0.5f) ? 1.0f : -1.0f;
            break;

        case LFO_SAMPLE_HOLD:
            // Sample & hold: random steps at LFO rate
            lfo->sh_timer += phase_inc;
            if (lfo->sh_timer >= 1.0f) {
                lfo->sh_timer -= 1.0f;
                // Generate random value [-1, 1]
                lfo->sh_value = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
            }
            lfo->value = lfo->sh_value;
            break;
    }

    return lfo->value;
}

// ============================================================================
// MIDI NOTE TO FREQUENCY
// ============================================================================

float midi_to_freq(uint8_t note) {
    // A4 (note 69) = 440 Hz
    return 440.0f * powf(2.0f, (note - 69) / 12.0f);
}

// ============================================================================
// PARAPHONIC SYNTH CONTROL
// ============================================================================

void synth_init(paraphonic_synth_t *s) {
    memset(s, 0, sizeof(paraphonic_synth_t));

    // Initialize ADSR envelopes (shared by all oscillators, controlled by CC82/83/85/17)
    adsr_init(&s->amp_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
    adsr_init(&s->filter_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);

    // Initialize filters (shared)
    svf_init(&s->filter_lp);  // Low-pass filter
    svf_init(&s->filter_hp);  // High-pass filter

    // Initialize LFO
    lfo_init(&s->lfo, lfo_rate);

    // Initialize drift LFOs (one per oscillator for analog feel)
    for (int i = 0; i < MAX_VOICES; i++) {
        drift_lfo_init(&s->oscillators[i].drift_lfo, i);
    }

    s->osc_type = OSC_SAW; // Default to saw wave
    s->pitch_bend = 0.0f;
    s->velocity = 100;
    s->active_count = 0;
}

void synth_note_on(paraphonic_synth_t *s, uint8_t note, uint8_t velocity) {
    // Count currently held notes
    int held_count = 0;
    for (int i = 0; i < MAX_VOICES; i++) {
        if (s->oscillators[i].held) held_count++;
    }

    // Check if this note is already held (retrigger)
    for (int i = 0; i < MAX_VOICES; i++) {
        if (s->oscillators[i].held && s->oscillators[i].note == note) {
            // Retrigger: reset all phases
            s->oscillators[i].phase = 0.0f;
            s->oscillators[i].sub_phase = 0.0f;
            s->oscillators[i].unison_phase = 0.0f;
            s->velocity = velocity;
            ESP_LOGI(TAG, "Note ON (retrigger): %d (%.2f Hz) Vel: %d",
                     note, s->oscillators[i].frequency, velocity);
            return;
        }
    }

    // Find a free (not held) oscillator
    for (int i = 0; i < MAX_VOICES; i++) {
        if (!s->oscillators[i].held) {
            s->oscillators[i].active = true;
            s->oscillators[i].held = true;
            s->oscillators[i].note = note;
            s->oscillators[i].frequency = midi_to_freq(note);
            s->oscillators[i].phase = 0.0f;
            s->oscillators[i].sub_phase = 0.0f;
            s->oscillators[i].unison_phase = 0.0f;
            s->velocity = velocity;

            held_count++;
            ESP_LOGI(TAG, "Note ON: %d (%.2f Hz) Vel: %d [%d held]",
                     note, s->oscillators[i].frequency, velocity, held_count);

            // Trigger envelopes on first held note
            if (held_count == 1) {
                adsr_note_on(&s->amp_env);
                adsr_note_on(&s->filter_env);
            }
            return;
        }
    }

    ESP_LOGW(TAG, "All oscillators busy, note %d ignored", note);
}

void synth_note_off(paraphonic_synth_t *s, uint8_t note) {
    // Find the oscillator playing this note and release it
    for (int i = 0; i < MAX_VOICES; i++) {
        if (s->oscillators[i].held && s->oscillators[i].note == note) {
            // Mark as no longer held (but keep active for release)
            s->oscillators[i].held = false;

            // Count remaining held notes
            int held_count = 0;
            for (int j = 0; j < MAX_VOICES; j++) {
                if (s->oscillators[j].held) held_count++;
            }

            ESP_LOGI(TAG, "Note OFF: %d [%d held]", note, held_count);

            // Trigger envelope release when last note is released
            if (held_count == 0) {
                adsr_note_off(&s->amp_env);
                adsr_note_off(&s->filter_env);
                ESP_LOGI(TAG, "Envelope RELEASE triggered");
            }
            return;
        }
    }
}

// ============================================================================
// AUDIO GENERATION TASK
// ============================================================================

void audio_generation_task(void *arg) {
    ESP_LOGI(TAG, "Audio task started (Core %d)", xPortGetCoreID());
    ESP_LOGI(TAG, "Latency: %.1f ms", (DMA_BUF_COUNT * DMA_BUF_LEN * 1000.0f) / SAMPLE_RATE);

    int16_t samples[DMA_BUF_LEN * 2]; // Stereo
    size_t bytes_written;

    while (1) {
        // Parameter smoothing (once per buffer to reduce CPU)
        lfo_rate += 0.05f * (lfo_rate_target - lfo_rate);
        synth.lfo.rate_hz = lfo_rate;

        // Smooth analog drift and filter drive (CC16 and CC19)
        drift_depth += 0.02f * (drift_depth_target - drift_depth);
        filter_drive += 0.02f * (filter_drive_target - filter_drive);

        // Smooth thickness and derive mix values (CC77)
        thickness_current += 0.02f * (thickness_target - thickness_current);
        // Use squared curves for subtle low values
        float thickness_sq = thickness_current * thickness_current;
        unison_mix = thickness_sq * 0.5f;   // 0 to 0.5 at full thickness
        sub_mix = thickness_sq * 0.35f;     // 0 to 0.35 at full thickness

        // Process LFO (once per buffer)
        float lfo_value = lfo_process(&synth.lfo);

        // Determine LFO destination based on CC93 mode
        float vibrato_mod = 0.0f, filter_mod = 0.0f;

        if (lfo_mode < 32) {
            // Mode 0-31: PWM modulation (disabled for CPU optimization)
            synth.lfo.waveform = LFO_TRIANGLE;
            // PWM modulation removed to reduce CPU load
        } else if (lfo_mode < 64) {
            // Mode 32-63: Filter wobble
            synth.lfo.waveform = LFO_SINE;
            filter_mod = lfo_value * lfo_depth * 3000.0f;  // ±3kHz
        } else if (lfo_mode < 96) {
            // Mode 64-95: Vibrato (pitch modulation)
            synth.lfo.waveform = LFO_SINE;
            vibrato_mod = lfo_value * lfo_depth * 0.05f;  // ±5 cents
        } else {
            // Mode 96-127: Sample & Hold filter
            synth.lfo.waveform = LFO_SAMPLE_HOLD;
            filter_mod = lfo_value * lfo_depth * 4000.0f;  // ±4kHz random steps
        }

        // Pre-calculate base pitch modulation (constant per buffer)
        float pitch_mod_base = synth.pitch_bend / 12.0f + vibrato_mod;

        // Set HP filter params once per buffer (doesn't change per sample)
        svf_set_params(&synth.filter_hp, hp_cutoff, 0.5f);

        // Pre-compute drift and pitch scaling per oscillator (once per buffer, not per sample)
        float osc_pitch_scale[MAX_VOICES];
        float max_drift_cents = drift_depth * 2.0f;  // 0.0 to 2.0 cents

        for (int v = 0; v < MAX_VOICES; v++) {
            if (synth.oscillators[v].active) {
                // Process analog drift LFO once per buffer (drift is slow)
                float drift_cents = drift_lfo_process(&synth.oscillators[v].drift_lfo, max_drift_cents);
                float pitch_mod_total = pitch_mod_base + (drift_cents / 12.0f);
                osc_pitch_scale[v] = powf(2.0f, pitch_mod_total);  // Expensive, but only once per buffer
            }
        }

        // Generate audio samples
        for (int i = 0; i < DMA_BUF_LEN; i++) {
            float mixed_osc = 0.0f;
            int osc_count = 0;

            // PARAPHONIC: Mix all active oscillators
            for (int v = 0; v < MAX_VOICES; v++) {
                para_osc_t *osc = &synth.oscillators[v];

                if (osc->active) {
                    // Use pre-computed pitch scale (includes pitch bend, vibrato, and drift)
                    float bent_freq = osc->frequency * osc_pitch_scale[v];
                    float phase_inc = bent_freq / SAMPLE_RATE;

                    // Generate main oscillator
                    float sample = generate_oscillator(osc->phase, phase_inc, synth.osc_type);

                    // Update main phase
                    osc->phase += phase_inc;
                    if (osc->phase >= 1.0f) osc->phase -= 1.0f;

                    // Generate sub and unison oscillators (only if thickness > 0)
                    float sub_sample = 0.0f;
                    float unison_sample = 0.0f;

                    if (thickness_current > 0.0001f) {
                        // Sub oscillator (square wave, -1 octave)
                        // Update phase for continuity, render if mix > threshold
                        float sub_freq = bent_freq * 0.5f;  // One octave down
                        float sub_phase_inc = sub_freq / SAMPLE_RATE;

                        if (sub_mix > 0.0001f) {
                            sub_sample = (osc->sub_phase < 0.5f) ? -1.0f : 1.0f;  // Square wave
                        }

                        // Always update phase (prevents discontinuities when CC77 changes)
                        osc->sub_phase += sub_phase_inc;
                        if (osc->sub_phase >= 1.0f) osc->sub_phase -= 1.0f;

                        // Unison detune oscillator (same waveform, detuned)
                        // Update phase for continuity, only render at moderate-to-high thickness
                        float unison_freq = bent_freq * detune_ratio;
                        float unison_phase_inc = unison_freq / SAMPLE_RATE;

                        if (unison_mix > 0.05f) {
                            // Only call expensive generate_oscillator when mix is significant
                            unison_sample = generate_oscillator(osc->unison_phase, unison_phase_inc, synth.osc_type);
                        }

                        // Always update phase (prevents discontinuities when CC77 changes)
                        osc->unison_phase += unison_phase_inc;
                        if (osc->unison_phase >= 1.0f) osc->unison_phase -= 1.0f;
                    }

                    // Mix main + sub + unison with appropriate gains (additive layering)
                    // At max thickness: main(1.0) + sub(0.35) + unison(0.5) = 1.85× amplitude
                    // Headroom is provided by reduced output scaling at final stage
                    sample = sample + (sub_sample * sub_mix) + (unison_sample * unison_mix);

                    // Add to mix
                    mixed_osc += sample;
                    osc_count++;
                }
            }

            // Process the mixed signal (paraphonic: shared envelope and filter)
            float output = 0.0f;
            if (osc_count > 0) {
                // Average the oscillators
                mixed_osc /= (float)osc_count;

                // Process shared envelopes
                float amp_env = adsr_process(&synth.amp_env);
                float filter_env = adsr_process(&synth.filter_env);

                // FILTER CHAIN: HP -> Drive -> LP

                // 1. High-pass filter (params already set above)
                output = svf_process_hp(&synth.filter_hp, mixed_osc);

                // 2. Filter drive (pre-filter saturation, controlled by CC19)
                output *= filter_drive;
                output = soft_clip(output);  // Soft saturation from drive

                // 3. Low-pass filter with envelope + LFO modulation
                float modulated_cutoff = filter_cutoff;
                modulated_cutoff += filter_env * filter_env_amount * 5000.0f;  // Envelope
                modulated_cutoff += filter_mod;  // LFO (wobble or S&H)
                svf_set_params(&synth.filter_lp, modulated_cutoff, filter_resonance);
                output = svf_process(&synth.filter_lp, output);

                // Apply shared amplitude envelope and velocity
                float velocity_scale = synth.velocity / 127.0f;
                output *= amp_env * velocity_scale;
            }

            // Apply soft saturation
            output = soft_clip(output);

            // Convert to 16-bit with headroom for thickness layering
            // Max amplitude at full thickness: 1.85× (main + sub*0.35 + unison*0.5)
            // Using 12000 provides ~2× headroom while maintaining good volume
            int16_t sample_i16 = (int16_t)(output * 12000.0f);

            // Stereo output (same on both channels)
            samples[i * 2] = sample_i16;
            samples[i * 2 + 1] = sample_i16;

            // DAC output for oscilloscope (8-bit, 0-255, center at 128)
            // Convert from signed 16-bit to unsigned 8-bit with DC offset
            uint8_t dac_value = (uint8_t)((sample_i16 / 256) + 128);
            dac_oneshot_output_voltage(dac_handle, dac_value);
        }

        // Write to I2S (blocking - provides natural pacing)
        i2s_channel_write(tx_handle, samples, sizeof(samples), &bytes_written, portMAX_DELAY);

        // Deactivate all oscillators when envelope release completes
        if (synth.amp_env.stage == ENV_IDLE) {
            bool any_active = false;
            for (int v = 0; v < MAX_VOICES; v++) {
                if (synth.oscillators[v].active && !synth.oscillators[v].held) {
                    synth.oscillators[v].active = false;
                    synth.oscillators[v].held = false;
                    synth.oscillators[v].phase = 0.0f;
                    synth.oscillators[v].sub_phase = 0.0f;
                    synth.oscillators[v].unison_phase = 0.0f;
                    any_active = true;
                }
            }
            if (any_active) {
                ESP_LOGI(TAG, "Release complete - oscillators deactivated");
            }
        }

        // Auto-save settings to NVS after 2 seconds of inactivity
        if (settings_need_save && last_cc_change_time > 0) {
            int64_t elapsed = esp_timer_get_time() - last_cc_change_time;
            if (elapsed >= 2000000) {  // 2 seconds in microseconds
                save_settings_to_nvs();
            }
        }

        // Note: No vTaskDelay needed - i2s_channel_write blocks until buffer ready,
        // providing jitter-free timing. Task watchdog reset happens automatically.
    }
}

// ============================================================================
// MIDI PROCESSING TASK
// ============================================================================

void midi_processing_task(void *arg) {
    ESP_LOGI(TAG, "MIDI task started (Core %d)", xPortGetCoreID());

    midi_event_t evt;

    while (1) {
        if (xQueueReceive(midi_queue, &evt, portMAX_DELAY) == pdTRUE) {
            uint8_t msg_type = evt.status & 0xF0;
            uint8_t channel = evt.status & 0x0F;

            switch (msg_type) {
                case MIDI_NOTE_ON:
                    if (evt.data2 > 0) {
                        synth_note_on(&synth, evt.data1, evt.data2);
                    } else {
                        // Note on with velocity 0 = note off
                        synth_note_off(&synth, evt.data1);
                    }
                    break;

                case MIDI_NOTE_OFF:
                    synth_note_off(&synth, evt.data1);
                    break;

                case MIDI_CONTROL_CHANGE:
                    // Logging temporarily re-enabled for debugging
                    ESP_LOGI(TAG, "MIDI CC: #%d = %d (Ch %d)", evt.data1, evt.data2, channel + 1);

                    if (evt.data1 == 16) {
                        // CC16: Analog Drift Depth (0.00 - 2.00 cents)
                        drift_depth_target = evt.data2 / 127.0f;
                        float max_cents = drift_depth_target * 2.0f;
                        ESP_LOGI(TAG, "  -> Drift Depth: %.2f cents", max_cents);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 19) {
                        // CC19: Filter Drive (1.0x - 6.0x pre-filter saturation)
                        filter_drive_target = 1.0f + (evt.data2 / 127.0f) * 5.0f;
                        ESP_LOGI(TAG, "  -> Filter Drive: %.2fx", filter_drive_target);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 74) {
                        // CC74: LP filter cutoff
                        filter_cutoff = 100.0f + (evt.data2 / 127.0f) * 8000.0f;
                        ESP_LOGI(TAG, "  -> LP Cutoff: %.1f Hz", filter_cutoff);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 71) {
                        // CC71: LP filter resonance
                        filter_resonance = evt.data2 / 127.0f;
                        ESP_LOGI(TAG, "  -> LP Resonance: %.2f", filter_resonance);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 76) {
                        // CC76: HP filter cutoff (20 Hz - 2000 Hz)
                        hp_cutoff = 20.0f + (evt.data2 / 127.0f) * 1980.0f;
                        ESP_LOGI(TAG, "  -> HP Cutoff: %.1f Hz", hp_cutoff);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 18) {
                        // CC18: LFO Rate (1 Hz - 120 Hz) - audio rate modulation!
                        lfo_rate_target = 1.0f + (evt.data2 / 127.0f) * 119.0f;
                        ESP_LOGI(TAG, "  -> LFO Rate: %.1f Hz", lfo_rate_target);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 93) {
                        // CC93: LFO Mode (0-31=PWM, 32-63=Filter, 64-95=Vibrato, 96-127=S&H)
                        lfo_mode = evt.data2;
                        const char *mode_name;
                        if (evt.data2 < 32) mode_name = "PWM";
                        else if (evt.data2 < 64) mode_name = "Filter Wobble";
                        else if (evt.data2 < 96) mode_name = "Vibrato";
                        else mode_name = "S&H Filter";
                        ESP_LOGI(TAG, "  -> LFO Mode: %s (%d)", mode_name, evt.data2);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 82) {
                        // CC82: Envelope Attack (0-5000 ms)
                        env_attack_ms = (evt.data2 / 127.0f) * 5000.0f;
                        adsr_update_params(&synth.amp_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
                        adsr_update_params(&synth.filter_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
                        ESP_LOGI(TAG, "  -> Env Attack: %.0f ms", env_attack_ms);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 83) {
                        // CC83: Envelope Decay (0-10000 ms)
                        env_decay_ms = (evt.data2 / 127.0f) * 10000.0f;
                        adsr_update_params(&synth.amp_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
                        adsr_update_params(&synth.filter_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
                        ESP_LOGI(TAG, "  -> Env Decay: %.0f ms", env_decay_ms);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 85) {
                        // CC85: Envelope Sustain (0-1)
                        env_sustain = evt.data2 / 127.0f;
                        adsr_update_params(&synth.amp_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
                        adsr_update_params(&synth.filter_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
                        ESP_LOGI(TAG, "  -> Env Sustain: %.2f", env_sustain);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 17) {
                        // CC17: Envelope Release (0-10000 ms)
                        env_release_ms = (evt.data2 / 127.0f) * 10000.0f;
                        adsr_update_params(&synth.amp_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
                        adsr_update_params(&synth.filter_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
                        ESP_LOGI(TAG, "  -> Env Release: %.0f ms", env_release_ms);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    } else if (evt.data1 == 77) {
                        // CC77: Thickness (Sub + Unison Mix) 0-127
                        thickness_target = evt.data2 / 127.0f;
                        // Preview derived mix values (actual smoothed values computed in audio loop)
                        float preview_sq = thickness_target * thickness_target;
                        float preview_unison = preview_sq * 0.5f;
                        float preview_sub = preview_sq * 0.35f;
                        ESP_LOGI(TAG, "  -> CC77 Thickness: %.2f (unison: %.2f, sub: %.2f)",
                                 thickness_target, preview_unison, preview_sub);
                        last_cc_change_time = esp_timer_get_time();
                        settings_need_save = true;
                    }
                    break;

                case MIDI_PITCH_BEND: {
                    // Pitch bend: 14-bit value, center = 8192
                    int bend_value = evt.data1 | (evt.data2 << 7);
                    synth.pitch_bend = ((bend_value - 8192) / 8192.0f) * 2.0f; // ±2 semitones
                    ESP_LOGI(TAG, "Pitch bend: %.2f semitones", synth.pitch_bend);
                    break;
                }
            }
        }
    }
}

// ============================================================================
// MIDI UART RECEIVE TASK
// ============================================================================

void midi_uart_task(void *arg) {
    ESP_LOGI(TAG, "MIDI UART task started - byte-by-byte low latency mode");

    uint8_t running_status = 0;
    uint8_t data_bytes[2] = {0};
    uint8_t data_count = 0;
    uint8_t expected_bytes = 0;

    // Wait a bit for queue to be fully initialized
    vTaskDelay(pdMS_TO_TICKS(100));

    if (midi_queue == NULL) {
        ESP_LOGE(TAG, "MIDI queue is NULL, UART task exiting!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "MIDI UART task ready");

    while (1) {
        uint8_t byte;

        // Read one byte at a time with blocking wait (minimal latency)
        int len = uart_read_bytes(MIDI_UART_NUM, &byte, 1, portMAX_DELAY);

        if (len != 1) continue;

        // Status byte (MSB set)
        if (byte & 0x80) {
            // Reset data collection
            data_count = 0;

            if (byte < 0xF0) {
                // Channel message - store as running status
                running_status = byte;
                uint8_t msg_type = byte & 0xF0;

                // Determine how many data bytes to expect
                if (msg_type == MIDI_NOTE_ON || msg_type == MIDI_NOTE_OFF ||
                    msg_type == MIDI_CONTROL_CHANGE || msg_type == MIDI_PITCH_BEND) {
                    expected_bytes = 2;
                } else {
                    expected_bytes = 1;  // Program change, channel pressure
                }
            } else {
                // System message - clear running status
                running_status = 0;
                expected_bytes = 0;
            }
            continue;
        }

        // Data byte (MSB clear)
        if (running_status == 0) {
            continue;  // No valid status, ignore data byte
        }

        // Collect data bytes
        if (data_count < 2) {
            data_bytes[data_count++] = byte;
        }

        // Once we have all expected bytes, send the message
        if (data_count >= expected_bytes) {
            midi_event_t evt = {0};
            evt.status = running_status;
            evt.data1 = (data_count >= 1) ? data_bytes[0] : 0;
            evt.data2 = (data_count >= 2) ? data_bytes[1] : 0;
            evt.padding = 0;

            // Send to MIDI processing task (non-blocking)
            BaseType_t result = xQueueSend(midi_queue, &evt, 0);
            if (result != pdTRUE) {
                // Only log occasionally to avoid stalling
                static uint32_t drop_count = 0;
                if ((++drop_count % 100) == 0) {
                    ESP_LOGW(TAG, "MIDI queue full (%lu drops)", drop_count);
                }
            }

            // Reset for next message
            data_count = 0;
        }
    }
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================

void app_main(void) {
    // Suppress all console logging except errors
    esp_log_level_set("*", ESP_LOG_ERROR);

    ESP_LOGI(TAG, "ESP32 Professional MIDI Synthesizer");
    ESP_LOGI(TAG, "Sample Rate: %d Hz, Latency: %.1f ms",
             SAMPLE_RATE, (DMA_BUF_COUNT * DMA_BUF_LEN * 1000.0f) / SAMPLE_RATE);

    // Initialize NVS flash for settings persistence
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS flash initialized");

    // Pre-compute detune ratio for unison oscillator (CPU optimization)
    detune_ratio = powf(2.0f, DETUNE_CENTS / 1200.0f);
    ESP_LOGI(TAG, "Detune ratio: %.6f (%.1f cents)", detune_ratio, DETUNE_CENTS);

    // Initialize sine table
    init_sine_table();
    ESP_LOGI(TAG, "Sine table initialized (%d samples)", SINE_TABLE_SIZE);

    // Initialize paraphonic synthesizer
    synth_init(&synth);
    ESP_LOGI(TAG, "Paraphonic synth initialized (%d oscillators)", MAX_VOICES);

    // Load saved settings from NVS and update envelopes
    load_settings_from_nvs();
    adsr_update_params(&synth.amp_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
    adsr_update_params(&synth.filter_env, env_attack_ms, env_decay_ms, env_sustain, env_release_ms);
    ESP_LOGI(TAG, "Applied loaded settings to synth parameters");

    // Create MIDI queue
    midi_queue = xQueueCreate(32, sizeof(midi_event_t));
    if (midi_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create MIDI queue!");
        return;
    }
    ESP_LOGI(TAG, "MIDI queue created");

    // Configure I2S
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_BUF_COUNT;
    chan_cfg.dma_frame_num = DMA_BUF_LEN;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws = I2S_LCK_PIN,
            .dout = I2S_DOUT_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_LOGI(TAG, "I2S configured (BCLK=%d, LCK=%d, DOUT=%d)",
             I2S_BCLK_PIN, I2S_LCK_PIN, I2S_DOUT_PIN);

    // Configure DAC for oscilloscope output
    dac_oneshot_config_t dac_cfg = {
        .chan_id = DAC_CHANNEL,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac_cfg, &dac_handle));
    ESP_LOGI(TAG, "DAC configured (GPIO%d for oscilloscope)", DAC_OUTPUT_PIN);

    // Configure UART for MIDI
    uart_config_t uart_config = {
        .baud_rate = MIDI_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(MIDI_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MIDI_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MIDI_UART_NUM, UART_PIN_NO_CHANGE, MIDI_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // UART tuning for minimal MIDI latency
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(MIDI_UART_NUM, 1));  // Interrupt on 1 byte
    ESP_ERROR_CHECK(uart_set_rx_timeout(MIDI_UART_NUM, 1));         // Minimal timeout

    ESP_LOGI(TAG, "UART configured (GPIO%d @ %d baud) - low latency mode", MIDI_RX_PIN, MIDI_BAUD_RATE);

    // Create tasks with error checking
    BaseType_t result;

    // Audio task: highest priority (20), pinned to core 1
    result = xTaskCreatePinnedToCore(audio_generation_task, "audio", 4096, NULL, 20, NULL, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task!");
        return;
    }
    ESP_LOGI(TAG, "Audio task created (priority 20)");
    vTaskDelay(pdMS_TO_TICKS(50));  // Let task start

    // MIDI UART task: high priority (14), pinned to core 0 - process bytes immediately
    result = xTaskCreatePinnedToCore(midi_uart_task, "midi_uart", 3072, NULL, 14, NULL, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MIDI UART task!");
        return;
    }
    ESP_LOGI(TAG, "MIDI UART task created (priority 14)");
    vTaskDelay(pdMS_TO_TICKS(50));  // Let task start

    // MIDI processing task: medium-high priority (13), pinned to core 0
    result = xTaskCreatePinnedToCore(midi_processing_task, "midi_proc", 3072, NULL, 13, NULL, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MIDI processing task!");
        return;
    }
    ESP_LOGI(TAG, "MIDI processing task created (priority 13)");

    ESP_LOGI(TAG, "=== READY ===");
    ESP_LOGI(TAG, "Mode: Paraphonic (%d osc) | Waveform: %s | Cutoff: %.0f Hz | Resonance: %.2f",
             MAX_VOICES, osc_names[synth.osc_type], filter_cutoff, filter_resonance);
}
