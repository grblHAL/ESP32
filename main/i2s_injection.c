/* I2S motor injection renderer. Part of grblHAL ESP32, GPLv3 or later. */
#if STEP_INJECT_STREAM
#include <string.h>
#include "i2s_injection.h"

#define SAMPLE_US 4u

bool i2s_injection_begin(i2s_injection_t *s, const injection_motion_t *motion,
                         uint32_t step_mask, uint32_t direction_mask,
                         uint32_t direction_bits, uint32_t step_idle_bits,
                         uint32_t pulse_samples, uint32_t direction_delay_us)
{
    if(s->active || !motion->id || !motion->next || !motion->requested_steps || !pulse_samples)
        return false;

    memset(s, 0, sizeof(*s));
    s->motion = *motion;
    s->step_mask = step_mask;
    s->direction_mask = direction_mask;
    s->direction_bits = direction_bits & direction_mask;
    s->step_idle_bits = step_idle_bits & step_mask;
    s->pulse_samples = pulse_samples;
    s->active = true;
    // The whole motion is accepted; this one pending event slot cannot fail.
    s->event = motion->next(motion->context);
    s->due_us = s->event.delay_us;
    if(s->due_us < direction_delay_us)
        s->due_us = direction_delay_us;
    return true;
}

bool i2s_injection_render(i2s_injection_t *s, uint32_t *samples, uint32_t count,
                          i2s_injection_checkpoint_t *checkpoint)
{
    checkpoint->valid = false;
    if(!s->active)
        return true;

    for(uint32_t i = 0; i < count; i++) {
        bool falling = false;
        // A pulse is confirmed only after an inactive sample has been emitted.
        if(s->high_samples == 1) {
            s->high_samples = 0;
            s->completed++;
            falling = true;
        } else if(s->high_samples)
            s->high_samples--;

        if(!s->terminal && s->sample_time_us >= s->due_us) {
            if(s->event.step) {
                if(s->high_samples || falling || s->generated >= s->motion.requested_steps) {
                    s->fault = true; // rate exceeds pulse + inactive-sample capacity
                    return false;
                }
                s->high_samples = s->pulse_samples;
                s->generated++;
            }
            if(s->event.last)
                s->terminal = true;
            else {
                s->event = s->motion.next(s->motion.context);
                // Accumulate requested times, not rounded per-step intervals.
                s->due_us += s->event.delay_us;
            }
        }

        uint32_t bits = s->direction_bits | s->step_idle_bits;
        if(s->high_samples)
            bits ^= s->step_mask;
        samples[i] = (samples[i] & ~(s->step_mask | s->direction_mask)) | bits;
        s->sample_time_us += SAMPLE_US;
    }

    checkpoint->id = s->motion.id;
    checkpoint->completed_steps = s->completed;
    checkpoint->last = s->terminal && !s->high_samples;
    checkpoint->valid = true;
    return true;
}

bool i2s_injection_confirm(i2s_injection_t *s, i2s_injection_checkpoint_t *checkpoint,
                           uint32_t generation, injection_progress_t *progress)
{
    if(!checkpoint->valid || checkpoint->generation != generation)
        return false;
    checkpoint->valid = false;
    if(!s->active || checkpoint->id != s->motion.id || checkpoint->completed_steps < s->confirmed)
        return false;
    s->confirmed = checkpoint->completed_steps;
    *progress = (injection_progress_t){s->motion.id, s->confirmed,
                                     checkpoint->last ? Injection_Completed : Injection_Progress};
    if(checkpoint->last)
        s->active = false;
    return true;
}
#endif
