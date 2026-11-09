# CV-30 laser control box: pin map

Recovered 2026-11-06 by disassembling Creality's firmware `CV-30-3DPrint-V3_0_24`
(ESP32-S2, Grbl_Esp32 1.3a on ESP-IDF 4.4.2), then checked on a real box
(PCB "V2.2 S2", 2023) running grblHAL: steppers, limit switches and the laser
output were all verified, the laser output with a meter. Addresses are in the
stock app as loaded (ota_0).

| Function | GPIO | How it's used | Where found |
|---|---|---|---|
| X step | 36 | output; pulse = `set_stepper_pins_on(mask ^ $2)` | 0x4008d560, called from the stepper timer ISR 0x40027400 |
| X direction | 35 | output; `set_direction_pins(st.dir_outbits)` | 0x4008d48c, same ISR |
| Y step | 7 | as X step, axis bit 1 | 0x4008d560 |
| Y direction | 6 | as X direction, axis bit 1 | 0x4008d48c |
| Steppers disable | 16 | high = motors off (`$4` inverts) | 0x4008e048 |
| Laser PWM | 37 | LEDC timer 0 / channel 0, **1 kHz, 10-bit, inverted**: duty = 1023 − power | 0x4008d0f0, 0x4008d3c8 |
| Laser enable | 38 | output, **active-high**: drives Q1, which switches the laser connector's VCC (24 V). Metered: laser off = 0 V, on = 24 V | 0x40087c48, 0x4008d3e4, 0x4008ae04 |
| X limit (min) | 40 | input, pull-up, any-edge interrupt; `limit_pins` table `[40,255]` | 0x40089240, table 0x3ffc3b60 |
| Y limit (min) | 42 | as X; `[42,255]`. No max switches, no Z | same |
| Front button | 33 | input, pull-up (`key_handler_task`) | 0x4008ec74 |
| Buzzer | 9 | output, toggled by `beep_task` | 0x4008fd84 |
| Status LED | 34 | LEDC timer 2 / channel 5, 5 kHz, 13-bit, hardware fade | 0x4008eef4 |
| microSD MOSI | 15 | SPI3 (`spi_bus_initialize` host 2) | 0x4009180c |
| microSD MISO | 2 | | |
| microSD SCLK | 14 | | |
| microSD CS | 13 | `sdspi_device_config` {host 2, cs 13, no CD/WP/INT} | DROM 0x3f006384 |
| USB D− / D+ | 19 / 20 | native USB-OTG (TinyUSB CDC in stock) | 0x4009615c |

## Laser connector
3 pins: **VCC / GND / PWM**. VCC is the 24 V supply, switched by Q1 (GPIO 38),
so the module has no power at all while the laser output is off. PWM is a
0-5 V signal at 1 kHz; inverted at the GPIO by the board, so grblHAL runs with
`$16` PWM-invert set. Metered with the module unplugged: `M5` 0 V / 0 V,
`M3 S0` 24 V / 0 V, `S100` 24 V / 0.5 V average, `S1000` 24 V / 5 V.

## Motor connectors
4 pins per axis, one coil on pins 1-2 and the other on 3-4 (TMC2208,
standalone mode). Motors with 6-pin connectors vary in where their coils
are; if a motor only buzzes or does nothing, check its coil pairs with a
meter (a coil reads a few ohms) and re-pin its cable so each coil lands on
1-2 or 3-4. `$CVSTEP` steps one axis directly for this kind of test.

## Stock settings worth keeping
`$0=10` µs step pulse, `$2=0`, `$3=2` (Y direction inverted), `$4=0`,
`$22=1` homing on, `$23=2` homing direction, `$30=1000` S max, `$32=1` laser
mode, `$100=80.008` / `$101=80.000` steps/mm, `$110/$111=10000` mm/min,
`$120/$121=500` mm/s², `$130=220` / `$131=240` travel.

## Flash layout (stock V3.0.24)
nvs 0x9000 16K · otadata 0xd000 8K · phy_init 0xf000 4K · ota_0 0x10000 1408K ·
ota_1 0x170000 1408K · storage (FAT) 0x2d0000 1M. No secure boot, no flash
encryption, download modes not disabled.

## Open questions
- Other board revisions and stock versions: only PCB V2.2 S2 with stock
  V3.0.24 has been tested.
