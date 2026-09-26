# Testing finite I2S injection

[Architecture and supported configuration](i2s-injection.md) define the behavior
under test. Use matching core and driver revisions; record any local changes.

## Reproducible host tests

From the ESP32 repository root, with Python 3 and an available host C compiler:

```sh
python main/grbl/tests/stepper2/run.py --cc gcc
python tests/i2s_injection/run.py --cc gcc
python tests/i2s_injection/check_ownership.py --cc gcc
```

The runners also accept Clang or TinyCC through `--cc`. They compile production
C functions with mock dependencies. These project test harnesses exercise
software behavior on the host. The renderer suite uses the actual portable renderer.

| Check | Expected evidence / coverage |
|---|---|
| Direct profile regression | 46,122 scenarios; before/after refactor serviced trace `17d7cd7b62360fa8` |
| Stream renderer | 771 scenarios; finite counts/directions, inversion, buffer boundaries and full-pulse tails |
| Stream lifecycle | Stale/duplicate reports, rejection/retry, busy state, controlled stop and reset |
| Ownership harness | Regression check for handing Z back to normal motion; inspect its pass/fail output |

The direct results were obtained at core `e07dffa`, `dff5583` and `f628524`, each
with zero failures using the same final suite. Stream results were obtained at
core `f628524` and ESP32 `739b0e9`, with zero failures. Host execution used TinyCC
0.9.27. The GCC commands above provide another supported way to run the suites.

Six target compilations of the complete production `stepper2.c` passed with
matching headers and Xtensa ESP32 GCC 8.4.0: `-O0`/`-O2` for the refactor with
stream disabled and for the final version with stream disabled/enabled. No
diagnostics were emitted. This checks target compilation of that unit; it does
not establish full firmware linking, hardware execution or complete RTOS
concurrency correctness. The complete firmware build below supplies a separate integration check.

## Complete firmware build

The `dlc32-stream` configuration passed a complete firmware build, including
linking, with ESP32 `739b0e9`, core `d6a4f22` and Plasma `6c90199`. It enables
Plasma and finite I2S stream injection using the standard MKS DLC32 V2 board map.
The final core and Plasma submodule selections add documentation to the tested
production code; the production sources are unchanged.

Tools: PlatformIO Core 6.2.0, Espressif 32 platform 5.3.0,
`framework-espidf@3.40403.0`, Xtensa ESP32 GCC 8.4.0 (2021r2-patch5).
The framework package identifies ESP-IDF 4.4.3 in its package and source metadata;
its bundled `version.txt` contains 4.4.2. The package identifier above records the
exact distribution used.

The [build configuration](../tests/i2s_injection/platformio.ini) retains the
successful environment's flags and embedded-file inputs. From the repository root:

```sh
pio run -c tests/i2s_injection/platformio.ini -e dlc32-stream
```

This configuration enables the production integration and disables development
test instrumentation. It supplies a reproducible compilation/linking check;
application input assignment and runtime board tests have their own setup.
The successful build emitted warnings, including the selected spindle's missing
direction output and an overwritten Plasma default initializer.

## Firmware scenario matrix

The following are a reusable integration plan, not claims of fresh hardware
results or a supplied board-specific stimulus fixture.

| Scenario | Stimulus | Observe |
|---|---|---|
| Finite correction | Request a known signed step count | Full output pulses and confirmed delta agree at normal completion |
| Direction reversal | Complete positive and negative corrections in sequence | DIR setup, inversion and signed accounting |
| Concurrent motion | Move planner axes other than Z while injecting | Unrelated sample bits and planner motion remain correct |
| Controlled stop | Request stop during acceleration/cruise | Braking, draining and terminal state; endpoint may differ |
| Reset / fault | Interrupt a queued correction | No stale update accepted as a new motion; position uncertainty handled |
| Handover | Finish correction, then request normal Z motion | Offset synchronized once before release; normal Z remains usable |
| THC inhibit / arc loss | Exercise the application's configured input path | Document the resulting stop/hold path and confirmed position |

There is no new dedicated production injection G-code in this series. The historical simulation adapter does provide custom M790-M795 test commands; see the [command reference and executed simulation cases](i2s-injection-simulation.md). Injection is requested
through the secondary-stepper API by the application. A `G1` command alone is a
planner movement, not proof of stream injection. A reproducible THC G-code test
also needs a specified application mode, input stimulus, coordinate setup and
expected correction. Machine-specific programs, calibration values and private
board fixtures are not bundled here. Add a portable stimulus fixture with its
own documented build options before describing it as an available firmware test.

Record axis steps/unit, acceleration and maximum rate, pulse width/delay and
inversion, stream flags, application mode, stimulus sequence, expected counts,
firmware/compiler revisions and the observed result. Use the existing Plasma
settings documentation rather than copying one machine's voltage/PID settings.

## Measuring EOF-to-STEP latency

A reproducible measurement starts with defined observation points. An ISR timestamp, a debug GPIO
written in that ISR and the hardware EOF event are different reference points.
Likewise, an internal I2S sample transition and the signal at the external driver
input can be separated by serialization, latching and interface propagation.

1. Record exact firmware revisions, build flags and the measurement topology.
2. Capture a documented EOF reference and the external STEP signal on a common
   timebase. State instrumentation overhead and any timestamp clock conversion.
3. Identify the specific pulse and descriptor associated with the reference;
   this association makes the interval traceable to a specific output event.
4. Define the signed interval as `t_STEP - t_EOF_reference`, the STEP polarity
   and whether the first or another pulse is being measured.
5. Repeat under stated idle and concurrent-motion conditions. Report sample
   count, minimum, maximum, distribution and acquisition resolution.
6. Distinguish this interval from request-to-output latency and from mechanical
   response. Also compare total complete pulses against confirmed counts.

Historical approximate measurements and the qualified DIRECT comparison are recorded in [simulation and timing evidence](i2s-injection-simulation.md). No universal measured latency value is claimed by this document. A measurement
from a particular circuit must be accompanied by its approved configuration and
method before publication. The code-derived 4 microsecond sample interval is a
different quantity from end-to-end latency.
