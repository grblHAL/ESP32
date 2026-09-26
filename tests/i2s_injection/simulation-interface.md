# Simulation adapter: custom test commands

The development test adapter implements these commands in
`main/testing/plasma_phase1.inc`; M795 delegates to
`main/testing/overall_delay.c`. The interface controls simulated inputs, records
motion checkpoints and provides optional physical-capture diagnostics.

The portable adapter is planned for subsequent test-environment releases. This
reference accompanies that work and explains the commands used in the
[simulation scenarios](../../doc/i2s-injection-simulation.md). Board-specific
ADC/ARC drive and static STEP diagnostics are identified separately in the table
so their dependencies remain visible.

## Custom M-code reference

These are test-build extensions, not newly standardized production G-code.
`THC_TEST_ENABLE` gates the simulation adapter; `THC_OVERALL_DELAY` additionally
gates M795. Commands are parser-synchronized and the adapter requires torch off,
secondary Z drained and no active abort. Program the complete stimulus before M3.

| Command | Implemented test meaning |
|---|---|
| `M790 P0` | Clear telemetry, repetition state, voltage schedule and scheduled arc loss |
| `M790 P1` | Begin a recorded repetition; requires initialized voltage THC |
| `M790 P2` | Record the endpoint and close the repetition |
| `M790 P3` | Print configuration and recorded results after closing the repetition |
| `M790 P4` | Read the physical ADC port directly, bypassing simulated voltage; diagnostic only |
| `M790 P5` | Report physical/logical ARC input diagnostics; target details vary |
| `M790 P6 Q0/1/2` | Historical target-specific ARC pin drive diagnostic; excluded from portable simulation use |
| `M791 P0 Q0` | Select real ARC and voltage inputs |
| `M791 P1 Q1` | Simulated logical ARC=true, physical voltage input |
| `M791 P2 Q1` | Physical ARC input, simulated voltage |
| `M791 P3 Q1` | Both inputs simulated, logical ARC=true |
| `M792 Poffset Qtime Rgain` | Queue absolute voltage offset in V at time in ms; model gain in V/mm |
| `M793 Ptag` | Record a named checkpoint, integer tag 1-254 |
| `M794 Ptime` | Schedule simulated ARC loss in ms; P0 disables it |
| `M795 P0 / P1 / P2` | Capture-fixture diagnostic: static STEP low / high / restore configured idle |
| `M795 P3` | Arm physical edge capture for ordinary Z movement |
| `M795 P4` | Stop capture and print its report |
| `M795 P5` | Arm capture for an isolated THC correction |

For M791, P is a two-bit mask: bit 0 simulates ARC OK, bit 1 simulates voltage;
Q is the simulated logical ARC value (0 or 1), not a GPIO voltage level.
M792 accepts P from -20 to +20 V, integer Q from 0 to 60000 ms, and R greater
than zero up to 1000 V/mm. The maximum is 16 events, with increasing Q and a
constant R. Q0 starts a new schedule. P is an absolute offset, not an increment.
A used schedule is replaced by the next M792. The clock starts on the first
active THC evaluation after torch start, not on receipt of the command.
M794 uses that same clock. M790 P6 and M795 pin operations require the specific
diagnostic/capture fixture; they are not general board-independent commands.
