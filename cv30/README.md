# grblHAL for the Creality CV-30 laser control box

Open-source [grblHAL](https://github.com/grblHAL) firmware for the **CV-30**,
the ESP32-S2 control box that Creality sells with its laser module kits
(the box with a power switch, one front button, a microSD slot and USB-C).

It replaces the stock firmware (Grbl_Esp32 1.3a) with a current grblHAL and
adds:

- **Wi-Fi**: LightBurn and LaserGRBL over the network (telnet, port 23), plus
  websocket, FTP and WebDAV for putting files on the microSD card.
- **Wireless firmware updates**: upload a new build and install it with
  `$UPDATE`, without opening the box or pressing any buttons.
- **No tools needed to install**: it goes on through Creality's own microSD
  updater, and the stock firmware is kept on the box as a permanent way back.
- **Three ways back to stock** (see [Going back to stock](#going-back-to-stock)).
- Everything else grblHAL offers: jobs streamed from the microSD card,
  laser mode, soft limits, homing, macros.

> **Not affiliated with Creality.** This is community firmware, provided
> under the GPL **without any warranty**. Installing it is at your own risk.

## Status

Tested on one box: PCB marked **"V2.2 S2"** (2023), stock firmware
**CV-30-3DPrint V3.0.24**, 5 W module, 24 V supply. Homing, limit switches,
motion and laser power have all been checked, the laser output with a meter.
Other board revisions and stock versions have not been tested yet; if you
try one, please report what you find.

## Laser safety

A 5 W diode laser can blind you instantly, including from reflections. Wear
glasses rated for your laser's wavelength (445-455 nm for Creality's blue
diode modules) whenever it can fire, never leave it running unattended, and
keep a way to put out a fire within reach. The box's power switch cuts both
the motors and the laser.

## Installing

You need a microSD card (FAT32) and the box running the stock firmware.

1. Download `CV-30-grblhal-<build>.bin` from the Releases page.
2. Copy it to the **root** of the microSD card. Make it the only file on the
   card whose name contains `CV-30` and ends in `.bin`.
3. With the box off, put the card in, then switch the box on. The stock
   firmware installs the file (it beeps) and restarts into grblHAL. This
   takes up to a minute.
4. Check it: connect USB and open a terminal (any baud rate) or LightBurn's
   console, and send `$I`. You should see
   `[BOARD:Creality CV-30 laser box (build N)]`.

The stock updater only installs a file name it has not installed before, so
every release has a new build number in its name.

### First-time setup

grblHAL starts with the same motion settings as the stock firmware (steps/mm,
speeds, 220 x 240 travel, homing), so a stock Creality setup behaves as
before. Changes from stock:

- Homing sets the machine origin to **0,0** at the home position.
- Homing moves X and Y together (there is no Z).
- The laser enable signal is driven correctly (the box switches the laser's
  24 V supply with it), so the laser module has no power at all while the
  laser is off.

If you built your own frame, check directions and homing before anything
else, with the laser unplugged: jog each axis a few mm (`$J=G91 X5 F500`) and
make sure + moves away from that axis's limit switch (fix with `$3`), then
home with `$H`, keeping a hand on the power switch (fix the homing direction
with `$23`). Set your real travel with `$130`/`$131` and turn on soft limits
with `$20=1`.

### Wi-Fi

Out of the box, grblHAL starts an access point:

- network **`CV30-Laser-Setup`**, password **`grblHALpwd`**
- the box is at **192.168.5.1**, telnet port 23

To join your own network instead, send these over USB or telnet (for example
from LightBurn's console):

```
$74=<your network name>
$75=<your Wi-Fi password>
$73=1
$REBOOT
```

`$73=1` is station only; `$73=3` keeps the setup access point as a fallback.
If you keep the access point, change its password with `$77=<new password>`.
`tools/wifi_setup.py` does the same over USB and asks for the password
without echoing it. The box then shows up as `cv30-laser.local` (mDNS) and
on your router's client list.

### LightBurn

Add a **GRBL** device:

- **Ethernet/TCP**, address `cv30-laser.local` (or the IP), port **23**,
  or **Serial/USB**
- Bed size: your travel (`$130` x `$131`)
- Origin: the corner your machine homes to
- **S-value max: 1000** (matches `$30`)

Only one telnet client can be connected at a time; close LightBurn before
using the tools in `tools/` over the network.

## Updating

Wirelessly, with Python 3:

```
python tools/ota.py CV-30-grblhal-<build>.bin [host]
```

It uploads the file to the microSD card over FTP, checks its size, sends
`$UPDATE=<file>` and checks the new build number in `$I` when the box comes
back. `$UPDATE` hands the file to the stock firmware's installer: the box
restarts into stock, installs it and restarts into the new build (about
30 s). Your settings are kept.

Or copy a new `CV-30-grblhal-<build>.bin` to the card, send `$STOCK`
and let the stock updater install it, as in [Installing](#installing).

## Going back to stock

The stock firmware stays in the first app slot and is never overwritten.

- **`$STOCK`**: restarts into the stock firmware.
- **Power-cycle escape**: switch the box on and, within 10 seconds, off
  again; do that **twice**, then switch it on a third time: it starts the
  stock firmware. Needs no computer, but **unplug USB first**: the box's
  processor is also powered from USB, so with USB connected the power switch
  does not restart it.
- **Crash guard**: if grblHAL crashes three times in a row on start, the box
  goes back to the stock firmware by itself.

To come back to grblHAL from stock, give the stock updater a
`CV-30-grblhal-<build>.bin` it has not installed before.

## Troubleshooting

- **A motor buzzes or jiggles but does not turn, or does nothing:** almost
  always the motor cable. The board wants one coil on connector pins 1-2 and
  the other on 3-4; motors with 6-pin connectors put their coils in different
  places. Measure the motor's wires (a coil reads a few ohms between its two
  wires) and re-pin the cable. `$CVSTEP=X400` (or `Y400`, add `-` for the
  other direction) steps one axis directly, bypassing the motion planner, to
  test the wiring. Always switch the power off before plugging or unplugging
  a motor: doing it live can destroy the driver.
- **Two motors on one axis** (split cable): both motors share one driver, so
  each gets half the current. If they turn in opposite directions (mirrored
  mounting), swap the two wires of **one** coil on **one** motor only.
- **LightBurn burns much lighter than expected:** check its speed units
  (mm/min vs mm/s) and that S-value max is 1000.
- **Can't connect over telnet:** another client (often LightBurn) already has
  the connection.
- **`$F` error 62 after swapping the card:** send `$FU` then `$FM` to
  remount it (`$UPDATE` does this itself).

## Building from source

ESP-IDF **5.5** (tested with v5.5.4), target `esp32s2`:

```
idf.py -B build-s2 set-target esp32s2
idf.py -B build-s2 build
```

The CV-30 board is selected automatically for the `esp32s2` target
(`BOARD_CV30`, on by default); the S2's settings come from
`sdkconfig.defaults.esp32s2`.

The image is `build-s2/grbl.bin`; rename it `CV-30-grblhal-<build>.bin`.
Bump `CV30_BUILD` in `main/CMakeLists.txt` for every image, so the stock
updater sees a new name and `$I` shows which build is running.

The CV-30 code is in `main/cv30.c` (boot guard, `$STOCK`, `$UPDATE`,
`$CVSTEP`) and `main/boards/cv30_map.h`; the board's pins and how they were
found are in [pinmap.md](pinmap.md). This fork also ports the grblHAL ESP32
driver to ESP-IDF 5 and adds the ESP32-S2.

## Credits and licence

- [grblHAL](https://github.com/grblHAL) by Terje Io and contributors.
- The stock firmware is Creality's build of
  [Grbl_Esp32](https://github.com/bdring/Grbl_Esp32) by Bart Dring.
- CV-30 port: Curtis Baldy.

GPL-3.0, like grblHAL.
