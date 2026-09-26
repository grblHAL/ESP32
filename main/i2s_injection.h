/* Bounded sample renderer and DMA checkpoints. GPLv3 or later. */
#ifndef _I2S_INJECTION_H_
#define _I2S_INJECTION_H_
#include "grbl/stepper_injection.h"

typedef struct {
    uint32_t generation;
    uint32_t id;
    uint32_t completed_steps;
    bool last;
    bool valid;
} i2s_injection_checkpoint_t;

typedef struct {
    injection_motion_t motion;
    injection_event_t event;
    uint64_t sample_time_us;
    uint64_t due_us;
    uint32_t high_samples;
    uint32_t pulse_samples;
    uint32_t generated;
    uint32_t completed;
    uint32_t confirmed;
    uint32_t step_mask;
    uint32_t direction_mask;
    uint32_t direction_bits;
    uint32_t step_idle_bits;
    bool active;
    bool terminal;
    bool fault;
} i2s_injection_t;

/* All accesses are serialized by the I2S driver, not by this portable engine. */
bool i2s_injection_begin(i2s_injection_t *state, const injection_motion_t *motion,
                         uint32_t step_mask, uint32_t direction_mask,
                         uint32_t direction_bits, uint32_t step_idle_bits,
                         uint32_t pulse_samples, uint32_t direction_delay_us);
bool i2s_injection_render(i2s_injection_t *state, uint32_t *samples, uint32_t count,
                          i2s_injection_checkpoint_t *checkpoint);
bool i2s_injection_confirm(i2s_injection_t *state, i2s_injection_checkpoint_t *checkpoint,
                           uint32_t generation, injection_progress_t *progress);
#endif
