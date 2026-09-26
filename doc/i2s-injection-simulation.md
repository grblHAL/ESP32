# Simulation-based validation and historical timing

[Architecture](i2s-injection.md) | [Host tests and measurement method](i2s-injection-testing.md)

## What was simulated, and what actually ran

The development test firmware ran the real Plasma control state machine, secondary
stepper, XY planner and ESP32 I2S/DMA/EOF path on an MKS DLC32 board. It replaced
ARC OK and voltage inputs with programmable stimuli. The simulated feedback was
`V = 120 + offset(t) + K * confirmed_Z_mm`, using EOF-confirmed secondary position.
The model has no physical ADC latency, electrical noise or plasma dynamics.

These firmware tests exercised the running controller with synthetic inputs;
the host C harness provides a separate layer of logic testing. The simulation build suppressed the
physical torch output, but STEP/DIR motion outputs remained functional. The
separate hardware-recording mode must not be assumed to have the same output
suppression. Neither mode is a virtual model of the whole machine.

**Availability:** the commands below exist in the historical test adapter
`main/testing/plasma_phase1.inc`, with `phase1.c` telemetry and optional
`overall_delay.c` capture. They are separate test-adapter additions to the production ESP32
implementation identified by `739b0e9`. This document records the existing test interface and results;
it does not claim that enabling one flag in the current public candidate installs
that harness. A portable adapter and its hooks must be packaged before these
examples can be advertised as runnable from the published series. Local board
wiring and diagnostic overrides are not part of the injection feature.

## Test-interface release placement

The [M790-M795 reference](../tests/i2s_injection/simulation-interface.md) belongs
to the simulation adapter where the commands are first implemented. The portable
subset is intended for subsequent test-environment PRs/releases. Current
production core/Plasma/ESP32 commits do not register it. The following scenarios
explain historical validation and future test use, not present-day availability
of the adapter in those production commits.

## Simulation use cases

The historical geometry for TC01-TC04 was a 100 x 60 mm rounded contour (R10)
with a 20 mm hole. The hole had no disturbance. Each repetition recorded
checkpoints and returned to the selected work-coordinate origin. The synthetic
model used K=10 V/mm, reference 120 V and Z resolution 400 steps/mm. These are
model/test parameters, not a calibration recommendation for a plasma source.

| Case | Stimulus and purpose | Evidence status |
|---|---|---|
| TC01 | +2 V at 500 ms; one finite correction during XY motion | Baseline scenario definition; ideal zero-tolerance offset is -0.2 mm, not a compulsory PID endpoint |
| TC02 | +4 V at 500 ms; longer finite correction and drain | Scenario definition; ideal zero-tolerance offset -0.4 mm |
| TC03 | Offsets 0, +2, +4, +6 V at 0, 500, 2000, 3500 ms | 50 repetitions at F3500, VAD=0: 150 disturbances/accepted requests; endpoint requested and confirmed totals agree |
| TC04 | Offsets 0, +2, -2, +2 V at the same times | Recorded complete single-run trace: signed moves -80,+96,+64,-96,-64; 400 total steps, net -80 |
| TC06 | +2 V at 500 ms and ARC loss at 600 ms | Recorded single run: accepted correction drains to 48/48; no new correction after ARC loss; native feed hold completes |

TC03 had 200 checkpoints, zero faults and zero dropped checkpoints, but 111
dropped detailed trace entries. It supports per-repetition totals, not a complete
per-motion trace. TC04's cited run retained all 2048 trace entries and had no
faults or dropped trace/checkpoints. The cited TC04 evidence covers one complete repetition.

TC06 completed its accepted correction about 60 ms after ARC loss and reached
hold with Z at rest about 470 ms after loss. Those are scenario response times,
not injection latency. The accepted move completed rather than being replaced
by a new motion after loss. The original report identifies the test configuration and scenario; its
uploaded firmware hash was not recorded. It did not
validate restart, probing, reignition or a physical plasma cut.

### Example stimulus setup: TC04

This is the stimulus portion of the historical harness scenario, not a complete
machine program and not executable on production firmware lacking the adapter.
After M5 and drained Z, prepare the repetition and schedule before the contour's
M3 and XY path:

```gcode
M790 P0
M791 P3 Q1
M790 P1
M792 P0 Q0 R10
M792 P2 Q500 R10
M792 P-2 Q2000 R10
M792 P2 Q3500 R10
```

After the contour, M5 and drained Z, `M790 P2` closes the repetition before a
separate return move; `M793 P3` records the return checkpoint and `M790 P3`
prints the report. For TC06, use the +2 V schedule and `M794 P600` before M3;
the recorded scenario ends in Hold, leaving recovery to a separate sequence.

The voltage THC setup used mode 1, valid logical ADC/ARC ports, a 0.1 s THC
delay, 1 V threshold, P/I/D=1/0/0 and VAD disabled. Port numbers depend on the
board's registered ports, not physical GPIO numbering. Successful voltage-control initialization establishes the starting condition
for the simulation. Evaluation follows signed endpoints, total full pulses,
faults, dropped records and return checkpoints; one voltage event can produce
several corrections.

## End-to-end latency measurement

Measurements taken during implementation traced the path from a changed simulated voltage
sample through THC and injection to the first physical STEP edge. The test
comprised 200 acquisitions on an MKS DLC32 V2.2,
50 acquisitions at each XY feed of 500, 850, 1100 and 3500 mm/min. A common
80 MHz capture timebase recorded software markers and the divided physical
Z STEP signal. The setup used simulated voltage input and physical STEP capture, with the
external driver and motor disconnected. Z resolution was
400 steps/mm, acceleration 150 mm/s^2, requested correction speed 50 mm/min;
input offset +2 V, K=10 V/mm, P/I/D=1/0/0, threshold 1 V and VAD=0.
Instrumentation overhead is included.

| Interval | Approximate mean | Observation / interpretation |
|---|---:|---|
| Changed simulated voltage sample to THC request | 3.929 ms | Measured; includes control-window scheduling |
| THC request to accepted submission | 0.026 ms | Measured |
| Submission to first-step recorded EOF reference | 12.837 ms | Measured; includes profile and buffered transport |
| Recorded EOF reference to first captured STEP edge | 0.083 ms | Measured first-step residual; range about 81.95-83.56 us |
| Submission to first captured STEP | 12.920 ms | Subtotal covering the preceding two transport intervals |
| Changed simulated sample to first captured STEP | 16.875 ms | Total; observed range about 14.009-20.834 ms |
| DIRECT first ramp interval at the same acceleration/resolution | 3.902 ms | Calculated from the timer profile, not a physical DIRECT run |
| Estimated added stream delay | 9.018 ms | 12.920 minus 3.902; direct scheduling/output overhead unmeasured |

For the direct profile, `0.676 * sqrt(2 / (150 * 400)) * 1e6` gives about
3902 us before integer handling. Holding the measured upstream control delays
fixed gives a model-only sample-to-edge total near 7.857 ms for DIRECT. It is
not a measured direct latency or an assertion that I2S is always 9 ms slower.

All 13,888 captured active edges matched the requested/generated/mapped/confirmed
counts in this campaign. Across *all* pulse positions, STEP minus the associated
EOF timestamp ranged from about -1.842 to +0.084 ms. The positive 83 us residual
is specific to the first-step observation; its applicability follows the measured first-step conditions.
EOF accounting is descriptor-based and is not an exact timestamp of each
external pin edge. The observed maximum is not a guaranteed worst-case bound.

The original source log for the latency measurements has SHA-256:
`fd535ce633be8434364c31b03b85eb8356e30c587f61fd6186e03a992c434c44`.
The configuration, stimulus and observation points above describe the measurement
scenario. The raw test records remain in the development evidence
archive. Physical ADC latency, mechanical response and additional target boards
would require their own measurements.
