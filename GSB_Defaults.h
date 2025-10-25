// ================================================================
// GSB_Defaults.h
// Auto-generated from GSB_Settings.txt
// Generated on 2025-10-29 22:09:30
// ================================================================
#pragma once

#define GRBL_DEFAULT_0    5.0f       // Step pulse time (µs). Typical usable range ~0–255 µs (driver-dependent). Shorter = faster edge.
#define GRBL_DEFAULT_1    255        // Stepper idle lock time (ms). 0–255; 255 = never disable steppers.
#define GRBL_DEFAULT_2    0          // Step pulse invert (bitmask). Bit per axis: X=1,Y=2,Z=4,A=8,B=16,C=32...
#define GRBL_DEFAULT_3    21         // Direction invert (bitmask). Same bit scheme as $2.
#define GRBL_DEFAULT_4    7          // Invert stepper enable (bitmask). Bit per motor output; 0 = normal, set bits to invert.
#define GRBL_DEFAULT_5    3          // Limit pins invert (bitmask). Bits per axis input; set to invert limit inputs.
#define GRBL_DEFAULT_6    0          // Probe input invert (boolean). 0 or 1.
#define GRBL_DEFAULT_8    2          // Ganged axes direction invert (bitmask). For dual motors (e.g., Y2 on Z), set bit for the ganged axis.
#define GRBL_DEFAULT_9    1          // Spindle PWM options (bitfield). Bit 0 enables PWM features; other bits driver/plugin-specific.
#define GRBL_DEFAULT_10   511        // Status report mask (bitmask). Sum of bits to include fields; driver extends beyond classic GRBL.
#define GRBL_DEFAULT_11   0.010f     // Junction deviation (mm). Positive float; typical 0.005–0.05 mm.
#define GRBL_DEFAULT_12   0.002f     // Arc tolerance (mm). Positive float; smaller = more accurate arcs.
#define GRBL_DEFAULT_13   0          // Report inches (boolean). 0=mm, 1=inches.
#define GRBL_DEFAULT_14   0          // Control input invert (bitmask). Bits for Start/Hold/Reset/Door/Estop; set to invert sense.
#define GRBL_DEFAULT_15   0          // Coolant invert mask (bitmask). Bit 0=flood, bit 1=mist. Set to invert outputs.
#define GRBL_DEFAULT_16   123        // Spindle invert mask (bitmask). Bits for spindle enable/dir/other; board/driver dependent mapping.
#define GRBL_DEFAULT_17   71         // Control pull-up disable mask (bitmask). Disable pull-ups per control input if set.
#define GRBL_DEFAULT_18   49         // Limit pull-up disable mask (bitmask). Disable pull-ups per limit input if set.
#define GRBL_DEFAULT_19   0          // Probe pull-up disable (boolean). 0=use pull-up, 1=disable.
#define GRBL_DEFAULT_20   0          // Soft limits enable (boolean). 0/1.
#define GRBL_DEFAULT_21   1          // Hard limits enable (boolean). 0/1; requires wired limit switches.
#define GRBL_DEFAULT_22   5          // Homing enable / options (bitmask). In grblHAL this is a bitfield; include 1 to enable.
#define GRBL_DEFAULT_23   7          // Homing direction mask (bitmask). Axis bits set = home toward positive; clear = toward negative.
#define GRBL_DEFAULT_24   500.000f   // Homing locate feed rate (mm/min). Positive float.
#define GRBL_DEFAULT_25   2000.000f  // Homing seek rate (mm/min). Positive float.
#define GRBL_DEFAULT_26   25         // Homing debounce (ms). 0–255 ms typical.
#define GRBL_DEFAULT_27   4.500f     // Homing pull-off (mm). Positive float.
#define GRBL_DEFAULT_28   0.100f     // G73 retract distance (mm). Non-negative float.
#define GRBL_DEFAULT_29   0.0f       // Step pulse delay (µs). 0–255 µs typical; adds delay before step edge.
#define GRBL_DEFAULT_30   1000.f     // Max spindle/laser value (RPM-equivalent or scale). Positive; used for S scaling.
#define GRBL_DEFAULT_31   0.f        // Min spindle/laser value. 0 or positive.
#define GRBL_DEFAULT_32   3          // Spindle/laser mode (enum/bitfield). Classic: 0=spindle, 1=laser; grblHAL extends for clones.
#define GRBL_DEFAULT_33   5000.000f  // PWM frequency (Hz). Typical laser 1–20 kHz; servo use ~50 Hz.
#define GRBL_DEFAULT_34   0.000f     // PWM off value (device units). Driver-dependent scaler (e.g., 0–255 or 0–1000).
#define GRBL_DEFAULT_35   0.000f     // PWM min value (device units). Driver-dependent scaler.
#define GRBL_DEFAULT_36   100.000f   // PWM max value (device units). Driver-dependent scaler.
#define GRBL_DEFAULT_37   0          // Steppers energize at power-up (boolean). 0/1.
#define GRBL_DEFAULT_38   0          // Spindle pulses-per-rev (PPR). 0 disables spindle-sync; else positive integer.
#define GRBL_DEFAULT_39   1          // Enable legacy/printable realtime cmds (boolean). 0/1.
#define GRBL_DEFAULT_40   0          // Jog soft-limited (boolean). 0/1; respect soft limits during jog.
#define GRBL_DEFAULT_41   0          // Parking enable (0=off, 1=on, 2=on w/override).
#define GRBL_DEFAULT_42   2          // Parking axis (bitmask). Choose axis to park: X=1,Y=2,Z=4 etc. (here Y).
#define GRBL_DEFAULT_43   1          // Homing locate cycles (passes). Integer 1–3.
#define GRBL_DEFAULT_44   4          // Homing cycle 1 axes (bitmask). Z=4, X=1, Y=2; sum bits for simultaneous homing.
#define GRBL_DEFAULT_45   3          // Homing cycle 2 axes (bitmask). Here X+Y.
#define GRBL_DEFAULT_46   0          // Homing cycle 3 axes (bitmask). 0 if unused.
#define GRBL_DEFAULT_47   0          // Homing cycle 4 axes (bitmask). Driver-dependent for extra axes.
#define GRBL_DEFAULT_48   0          // Homing cycle 5 axes (bitmask).
#define GRBL_DEFAULT_49   0          // Homing cycle 6 axes (bitmask).
#define GRBL_DEFAULT_50   5000.000f  // Jog step speed (mm/min). Positive float.
#define GRBL_DEFAULT_51   1000.000f  // Jog slow speed (mm/min). Positive float.
#define GRBL_DEFAULT_52   30000.000f // Jog fast speed (mm/min). Positive float.
#define GRBL_DEFAULT_53   8.000f     // Jog step distance (mm). Positive float.
#define GRBL_DEFAULT_54   1.000f     // Jog slow distance (mm). Positive float.
#define GRBL_DEFAULT_55   10.000f    // Jog fast distance (mm). Positive float.
#define GRBL_DEFAULT_56   5.000f     // Parking pull-out increment (mm). Positive float.
#define GRBL_DEFAULT_57   1000.000f  // Parking pull-out rate (mm/min). Positive float.
#define GRBL_DEFAULT_58   -10.000f   // Parking target (mm). Signed float.
#define GRBL_DEFAULT_59   3000.000f  // Parking fast rate (mm/min). Positive float.
#define GRBL_DEFAULT_60   0          // Restore overrides after parking (boolean). 0/1.
#define GRBL_DEFAULT_61   0          // Safety door options (bitmask). Bits define behavior; board/sender dependent.
#define GRBL_DEFAULT_62   0          // Sleep enable (boolean). 0/1.
#define GRBL_DEFAULT_63   2          // Feed hold actions (bitmask). Defines what to stop on hold; sender/driver dependent.
#define GRBL_DEFAULT_64   0          // Force initialization alarm on startup (boolean). 0/1.
#define GRBL_DEFAULT_65   0          // Probing flags (bitmask). Bit 0 allows feed override during probing; other bits extend behavior.
#define GRBL_DEFAULT_66   0.000f     // Linear spindle piecewise map 1 (RPM/PWM). Driver-dependent.
#define GRBL_DEFAULT_67   0.000f     // Linear spindle piecewise map 2.
#define GRBL_DEFAULT_68   0.000f     // Linear spindle piecewise map 3.
#define GRBL_DEFAULT_69   0.000f     // Linear spindle piecewise map 4.
#define GRBL_DEFAULT_70   38         // Network services (bitmask). Bits: 0=Telnet,1=WebSocket,2=HTTP,3=FTP,4=DNS,5=mDNS,6=SSDP,7=WebDAV.
#define GRBL_DEFAULT_73   1          // WiFi mode: 0=None, 1=STA, 2=AP, 3=AP+STA.
#define GRBL_DEFAULT_79   0          // WiFi AP channel (0–11 typical; region dependent).
#define GRBL_DEFAULT_100  26.667f    // X steps/mm. Positive float.
#define GRBL_DEFAULT_101  26.667f    // Y steps/mm. Positive float.
#define GRBL_DEFAULT_102  364.516f   // Z steps/mm. Positive float.
#define GRBL_DEFAULT_103  26.667f    // A steps/mm. Positive float.
#define GRBL_DEFAULT_104  364.516f   // B steps/mm. Positive float.
#define GRBL_DEFAULT_110  36000.000f // X max rate (mm/min). Positive float.
#define GRBL_DEFAULT_111  36000.000f // Y max rate (mm/min). Positive float.
#define GRBL_DEFAULT_112  3000.000f  // Z max rate (mm/min). Positive float.
#define GRBL_DEFAULT_113  36000.000f // A max rate (mm/min). Positive float.
#define GRBL_DEFAULT_114  3000.000f  // B max rate (mm/min). Positive float.
#define GRBL_DEFAULT_120  2000.000f  // X acceleration (mm/s^2). Positive float.
#define GRBL_DEFAULT_121  2000.000f  // Y acceleration (mm/s^2). Positive float.
#define GRBL_DEFAULT_122  300.000f   // Z acceleration (mm/s^2). Positive float.
#define GRBL_DEFAULT_123  2000.000f  // A acceleration (mm/s^2). Positive float.
#define GRBL_DEFAULT_124  300.000f   // B acceleration (mm/s^2). Positive float.
#define GRBL_DEFAULT_130  810.000f   // X travel (mm). Positive float; used by soft limits.
#define GRBL_DEFAULT_131  800.000f   // Y travel (mm).
#define GRBL_DEFAULT_132  300.000f   // Z travel (mm).
#define GRBL_DEFAULT_133  810.000f   // A travel (mm).
#define GRBL_DEFAULT_134  300.000f   // B travel (mm).
#define GRBL_DEFAULT_200  26.667f    // (Extended axes) Steps/mm for next axis set (A/B/C); same as $100+ for axes >3.
#define GRBL_DEFAULT_201  26.667f    // (Extended axes) Steps/mm.
#define GRBL_DEFAULT_202  364.516f   // (Extended axes) Steps/mm.
#define GRBL_DEFAULT_210  36000.000f // (Extended axes) Max rate (mm/min).
#define GRBL_DEFAULT_211  36000.000f // (Extended axes) Max rate (mm/min).
#define GRBL_DEFAULT_212  3000.000f  // (Extended axes) Max rate (mm/min).
#define GRBL_DEFAULT_220  2000.000f  // (Extended axes) Acceleration (mm/s^2).
#define GRBL_DEFAULT_221  2000.000f  // (Extended axes) Acceleration (mm/s^2).
#define GRBL_DEFAULT_222  300.000f   // (Extended axes) Acceleration (mm/s^2).
#define GRBL_DEFAULT_230  810.000f   // (Extended axes) Travel (mm).
#define GRBL_DEFAULT_231  800.000f   // (Extended axes) Travel (mm).
#define GRBL_DEFAULT_232  300.000f   // (Extended axes) Travel (mm).
#define GRBL_DEFAULT_301  0          // IP mode (iface #1). 0=Static, 1=DHCP, 2=AutoIP.
#define GRBL_DEFAULT_304  255.255.255.0f // Netmask (iface #1). IPv4 text.
#define GRBL_DEFAULT_305  23         // Telnet port (iface #1). 1–65535.
#define GRBL_DEFAULT_306  80         // HTTP port (iface #1). 1–65535.
#define GRBL_DEFAULT_307  81         // WebSocket port (iface #1). 1–65535.
#define GRBL_DEFAULT_308  21         // FTP port (iface #1). 1–65535.
#define GRBL_DEFAULT_321  0          // IP mode (iface #3). 0=Static, 1=DHCP, 2=AutoIP.
#define GRBL_DEFAULT_322  192.168.5.1f // IP address (iface #3). IPv4 text.
#define GRBL_DEFAULT_323  192.168.5.1f // Gateway (iface #3). IPv4 text.
#define GRBL_DEFAULT_324  255.255.255.0f // Netmask (iface #3). IPv4 text.
#define GRBL_DEFAULT_325  23         // Telnet port (iface #3). 1–65535.
#define GRBL_DEFAULT_326  80         // HTTP port (iface #3). 1–65535.
#define GRBL_DEFAULT_327  81         // WebSocket port (iface #3). 1–65535.
#define GRBL_DEFAULT_328  21         // FTP port (iface #3). 1–65535.
#define GRBL_DEFAULT_336  0          // DST active (boolean). 0/1.
#define GRBL_DEFAULT_338  0          // Trinamic driver (enum). Driver-specific if TMC features are enabled.
#define GRBL_DEFAULT_339  0          // Trinamic homing options (bitfield). Driver-specific.
#define GRBL_DEFAULT_340  0          // Spindle at-speed tolerance (%). 0 disables check; else 0–100.
#define GRBL_DEFAULT_341  3          // Tool change mode. 0=Manual,1=Manual+TouchOff,2=Semi-auto,3=Auto touch-off @ G59.3.
#define GRBL_DEFAULT_342  30.000f    // Tool-change probing distance (mm). Positive float.
#define GRBL_DEFAULT_343  25.000f    // Tool-change locate feed (mm/min). Positive float.
#define GRBL_DEFAULT_344  200.000f   // Tool-change search seek rate (mm/min). Positive float.
#define GRBL_DEFAULT_345  200.000f   // Tool-change probe pull-off rate (mm/min). Positive float.
#define GRBL_DEFAULT_346  1          // Tool-change options (bitmask). Bit 0 typically = restore position after M6.
#define GRBL_DEFAULT_347  0          // Dual-axis length fail percent (%). For auto-squaring checks.
#define GRBL_DEFAULT_348  0.000f     // Dual-axis length fail min (mm). Threshold.
#define GRBL_DEFAULT_349  0.000f     // Dual-axis length fail max (mm). Threshold.
#define GRBL_DEFAULT_370  0          // Invert I/O port inputs (bitmask). Driver-specific mapping.
#define GRBL_DEFAULT_372  0          // Invert I/O port outputs (bitmask). Driver-specific mapping.
#define GRBL_DEFAULT_376  1          // Rotary axes enabled (bitmask/enum). Driver-specific.
#define GRBL_DEFAULT_384  0          // Disable G92 persistence (boolean). 0=save/restore G92 offset, 1=do not persist.
#define GRBL_DEFAULT_392  4.000f     // Door spindle-on delay (s). Wait after safety door before re-enabling spindle.
#define GRBL_DEFAULT_393  1.000f     // Door coolant-on delay (s). Wait after safety door before re-enabling coolant.
#define GRBL_DEFAULT_394  0.000f     // Spindle on delay (s). Delay after M3/M4 before motion resumes.
#define GRBL_DEFAULT_395  0          // Spindle type / selection (enum). 0 = default; plugin/board dependent.
#define GRBL_DEFAULT_396  30         // WebUI timeout (minutes).
#define GRBL_DEFAULT_397  0          // WebUI auto-report interval (ms). 0=disabled.
#define GRBL_DEFAULT_398  100        // Planner buffer blocks. Typical 16–256; larger = smoother but more memory.
#define GRBL_DEFAULT_481  0          // Autoreport interval (ms). 0=disabled.
#define GRBL_DEFAULT_482  0          // Timezone offset (min). Integer minutes from UTC.
#define GRBL_DEFAULT_483  0          // Link fan to spindle (boolean/enum). Driver-dependent.
#define GRBL_DEFAULT_484  1          // Unlock after E‑Stop (boolean). 0/1.
#define GRBL_DEFAULT_485  0          // Enable tool persistence (boolean). 0/1.
#define GRBL_DEFAULT_486  0          // Offset lock (bitmask). Lock G54–G59 etc.; sender/driver dependent.
#define GRBL_DEFAULT_511  0          // SpindleEnable1 (port mapping). 0=none; driver/plugin-specific mapping for extra spindles.
#define GRBL_DEFAULT_520  0          // Spindle tool-start 0 mapping. Driver/plugin-specific.
#define GRBL_DEFAULT_522  1          // Spindle tool-start 2 mapping. Driver/plugin-specific.
#define GRBL_DEFAULT_523  0          // Spindle tool-start 3 mapping. Driver/plugin-specific.
#define GRBL_DEFAULT_650  0          // FS (filesystem) options (bitmask). Driver/plugin-specific (SD, LittleFS, WebDAV etc.).
#define GRBL_DEFAULT_651  1          // Stepper1 options (bitfield). Driver-specific per-motor settings.
#define GRBL_DEFAULT_652  1          // Stepper2 options (bitfield). Driver-specific.
#define GRBL_DEFAULT_653  1          // Stepper3 options (bitfield). Driver-specific.
#define GRBL_DEFAULT_655  1          // Stepper5 options (bitfield). Driver-specific.
#define GRBL_DEFAULT_656  1          // Stepper6 options (bitfield). Driver-specific.
#define GRBL_DEFAULT_657  1          // Stepper7 options (bitfield). Driver-specific.
#define GRBL_DEFAULT_659  1          // Stepper9 options (bitfield). Driver-specific.
#define GRBL_DEFAULT_665  1          // Stepper15 options (bitfield). Driver-specific.
#define GRBL_DEFAULT_671  0          // Home pins invert mask (bitmask). Bits per axis home input.
#define GRBL_DEFAULT_673  0.000f     // Coolant on delay (s). Delay before motion resumes after coolant enable.
#define GRBL_DEFAULT_674  0          // THC options (bitmask). Plasma-only.
#define GRBL_DEFAULT_675  0          // Macro ATC options (bitmask). If using ATC plugin.
#define GRBL_DEFAULT_676  15         // Reset actions (bitmask). What to clear at reset (overrides, offsets, etc.).
#define GRBL_DEFAULT_677  0          // Stepper‑spindle options (bitmask). For closed-loop/stepper-driven spindles.
#define GRBL_DEFAULT_678  0          // Relay port for toolsetter (mapping). Driver-specific.
#define GRBL_DEFAULT_679  0          // Relay port for 2nd probe (mapping). Driver-specific.
#define GRBL_DEFAULT_680  0          // Stepper enable delay (ms). Small delay after enabling steppers before motion.
#define GRBL_DEFAULT_709  0          // Spindle PWM options #1 (bitmask) for cloned spindle/laser. Driver-specific.
#define GRBL_DEFAULT_716  0          // Spindle invert mask #1 (bitmask) for cloned spindle/laser.
#define GRBL_DEFAULT_730  1000.000f  // RpmMax1 (clone) – max scale for secondary spindle/laser.
#define GRBL_DEFAULT_731  0.000f     // RpmMin1 (clone) – min scale for secondary spindle/laser.
#define GRBL_DEFAULT_732  2          // Mode1 (enum) for cloned spindle/laser. Driver-specific (e.g., PWM vs RPM mode).
#define GRBL_DEFAULT_733  50.000f    // PWM frequency #1 (Hz) for cloned spindle/laser.
#define GRBL_DEFAULT_734  2.000f     // PWM off value #1 (device units) for clone.
#define GRBL_DEFAULT_735  2.000f     // PWM min value #1.
#define GRBL_DEFAULT_736  15.000f    // PWM max value #1.
#define GRBL_DEFAULT_742  0          // Motor warnings enable (boolean). Driver-specific.
#define GRBL_DEFAULT_743  0          // Motor warnings invert (boolean).
#define GRBL_DEFAULT_744  0          // Motor faults enable (boolean).
#define GRBL_DEFAULT_745  0          // Motor faults invert (boolean).
#define GRBL_DEFAULT_750  9          // Action0 mapping (enum). Button/action mapping – driver/sender dependent.
#define GRBL_DEFAULT_751  13         // Action1 mapping (enum).
#define GRBL_DEFAULT_752  14         // Action2 mapping (enum).
#define GRBL_DEFAULT_753  10         // Action3 mapping (enum).
#define GRBL_DEFAULT_754  0          // Action4 mapping (enum).

// End of auto-generated defaults
// ================================================================
