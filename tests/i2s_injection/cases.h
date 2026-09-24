static i2s_injection_t renderer;
static unsigned locks;
static bool reject_submit;

static void check (bool value, const char *message)
{
    if(!value) { if(failures < 15) printf("FAIL: %s\n", message); failures++; }
}
static void lock (void) { locks++; }
static void unlock (void) { check(locks != 0, "balanced lock"); locks--; }
static bool supports (uint32_t axis) { return axis == 4; }
static bool submit (const injection_motion_t *motion)
{
    check(locks != 0, "submission is locked");
    return !reject_submit && i2s_injection_begin(&renderer, motion, 8, 16,
            motion->direction_mask ? 16 : 0, 0, 2, 4);
}
static void cancel (uint32_t id)
{
    if(renderer.active && renderer.motion.id == id) renderer.active = false;
}
static const stepper_injection_t transport = {supports, submit, cancel, lock, unlock};

static void configure (st2_motor_t *motor)
{
    memset(motor, 0, sizeof(*motor));
    memset(&renderer, 0, sizeof(renderer));
    settings.axis[0] = (axis_settings_t){400, 150*3600.0f, 500};
    motor->axis.bits = 4;
    motor->executor.stream = true;
    motor->on_stopped = stopped;
    hal.stepper.injection = &transport;
    st2_motor_config(motor, &settings.axis[0]);
    output_calls = callback_calls = 0;
    reject_submit = false;
    sys.position_lost = false;
}

static void finite (uint32_t count, bool negative, uint32_t frame_size)
{
    st2_motor_t motor;
    configure(&motor);
    // Use the direct executor as the timing reference, then compare actual
    // rendered edge times with its service times (at most one sample late).
    st2_motor_t reference = motor;
    reference.executor.stream = false;
    reference.on_stopped = NULL;
    clock_us = 0;
    uint64_t expected[128];
    uint32_t edge = 0;
    st2_motor_move(&reference, (float)count, 200, Stepper2_Steps);
    if(count == 1)
        expected[edge++] = 4; // explicit single step still needs DIR setup in DMA
    else while(st2_motor_running(&reference)) {
        uint32_t before = output_calls;
        clock_us += reference.profile.delay;
        motor_irq(&reference);
        if(output_calls != before) expected[edge++] = clock_us;
    }
    check(edge == count, "direct reference count");
    output_calls = callback_calls = 0;
    check(st2_motor_move(&motor, negative ? -(float)count : (float)count, 200, Stepper2_Steps), "stream move accepted");
    check(st2_get_position(&motor) == 0, "acceptance is not execution");
    uint32_t data[500];
    i2s_injection_checkpoint_t map = {0};
    uint32_t rises = 0, falls = 0, high_width = 0;
    bool high = false;
    unsigned limit = 100000;
    uint64_t sample_us = 0;
    while(renderer.active && --limit) {
        for(uint32_t i = 0; i < frame_size; i++) data[i] = 0xa5a55aa5u ^ (i << 6);
        map.generation++;
        int64_t before = st2_get_position(&motor);
        lock();
        check(i2s_injection_render(&renderer, data, frame_size, &map), "render fits pulse timing");
        unlock();
        check(st2_get_position(&motor) == before, "rendering cannot confirm position");
        for(uint32_t i = 0; i < frame_size; i++) {
            check((data[i] & ~24u) == ((0xa5a55aa5u ^ (i << 6)) & ~24u), "unowned output bits preserved");
            check(!!(data[i] & 16) == negative, "direction preserved");
            bool value = !!(data[i] & 8);
            if(value && !high) {
                check(rises < count && sample_us >= expected[rises] && sample_us - expected[rises] < 4,
                      "sample edge agrees with direct executor within 4us");
                rises++;
            }
            if(!value && high) { falls++; check(high_width == 2, "pulse spans exactly configured samples"); high_width = 0; }
            if(value) high_width++;
            high = value;
            sample_us += 4;
        }
        injection_progress_t progress;
        check(!i2s_injection_confirm(&renderer, &map, map.generation-1, &progress), "stale generation rejected");
        check(i2s_injection_confirm(&renderer, &map, map.generation, &progress), "EOF checkpoint accepted");
        check(progress.completed_steps == falls, "only full physical pulses confirmed");
        check(locks == 0, "notification outside transport lock");
        renderer.motion.notify(renderer.motion.context, &progress);
        check(st2_get_position(&motor) == (negative ? -(int64_t)falls : falls), "position follows EOF delta");
        check(!i2s_injection_confirm(&renderer, &map, map.generation, &progress), "duplicate EOF not applied twice");
    }
    check(limit != 0 && rises == count && falls == count, "exact complete move");
    check(!st2_motor_running(&motor) && output_calls == 0, "stream does not use direct output");
    st2_motor_run(&motor);
    check(callback_calls == 1, "foreground completion exactly once");
    tests++;
}

static injection_event_t fast_next (void *context)
{
    uint32_t *remaining = context;
    return (injection_event_t){1, true, --*remaining == 0};
}

static void transport_edges (void)
{
    memset(&renderer, 0, sizeof(renderer));
    uint32_t remaining = 1;
    injection_motion_t motion = {.id = 99, .axis_mask = 4, .requested_steps = 1, .context = &remaining, .next = fast_next};
    check(i2s_injection_begin(&renderer, &motion, 8, 16, 16, 8, 1, 4), "inverted one-step accepted");
    uint32_t data[500] = {0};
    i2s_injection_checkpoint_t map = {.generation = 42};
    check(i2s_injection_render(&renderer, data, 2, &map), "inverted pulse begins");
    check((data[0] & 24) == 24 && (data[1] & 24) == 16, "inverted edge and DIR setup");
    check(map.completed_steps == 0 && !map.last, "high pulse not yet complete");
    check(i2s_injection_render(&renderer, data, 1, &map), "inverted tail rendered");
    check((data[0] & 24) == 24 && map.completed_steps == 1 && map.last, "tail completes full inverted pulse");
    injection_progress_t progress;
    i2s_injection_checkpoint_t old_map = map;
    check(i2s_injection_confirm(&renderer, &map, 42, &progress), "final EOF");
    motion.id++;
    motion.requested_steps = remaining = 10;
    check(i2s_injection_begin(&renderer, &motion, 8, 16, 0, 0, 2, 4), "new ID accepted");
    check(!i2s_injection_confirm(&renderer, &old_map, 42, &progress), "previous motion cannot confirm new motion");
    check(!i2s_injection_render(&renderer, data, 500, &map) && renderer.fault, "pulse overlap reports fault");
    check(!map.valid, "failed render cannot publish a checkpoint");
    tests++;
}

int main (void)
{
    for(uint32_t n = 1; n <= 128; n++)
        for(unsigned sign = 0; sign < 2; sign++) {
            finite(n, sign, 1);
            finite(n, sign, 7);
            finite(n, sign, 495);
        }
    st2_motor_t motor;
    configure(&motor);
    reject_submit = true;
    check(!st2_motor_move(&motor, 40, 200, Stepper2_Steps), "backpressure rejects whole motion");
    check(motor.executor.generated == 0, "rejection does not advance generator");
    reject_submit = false;
    check(st2_motor_move(&motor, 1000, 200, Stepper2_Steps), "retry accepted");
    check(!st2_motor_move(&motor, 40, 200, Stepper2_Steps), "busy rejects second motion");
    uint32_t old_id = motor.executor.id;
    uint32_t data[495] = {0};
    i2s_injection_checkpoint_t map = {.generation = 1};
    check(i2s_injection_render(&renderer, data, 495, &map), "prepare before reset");
    motors = &motor;
    st2_reset();
    check(!renderer.active && !st2_motor_running(&motor) && motor.position_lost, "active reset invalidates pending motion");
    injection_progress_t stale = {old_id, 1000, Injection_Completed};
    st2_stream_progress(&motor, &stale);
    check(st2_get_position(&motor) == 0 && !motor.executor.completed_pending, "late completion ignored after reset");
    check(st2_set_position(&motor, 0), "explicit restoration clears lost state");
    st2_reset();
    check(!motor.position_lost, "idle reset retains known position");
    tests++;
    check(st2_motor_move(&motor, 1000, 200, Stepper2_Steps), "braking move accepted");
    check(st2_motor_stop(&motor), "controlled stop requested");
    unsigned limit = 100000;
    while(renderer.active && --limit) {
        map.generation++;
        check(i2s_injection_render(&renderer, data, 495, &map), "braking render");
        injection_progress_t progress;
        if(i2s_injection_confirm(&renderer, &map, map.generation, &progress))
            st2_stream_progress(&motor, &progress);
    }
    check(limit && motor.executor.confirmed < 1000 && motor.executor.confirmed > 0, "braking completes partial motion");
    tests++;
    transport_edges();
    printf("%u stream scenarios, %u failures\n", tests, failures);
    return failures ? 1 : 0;
}
