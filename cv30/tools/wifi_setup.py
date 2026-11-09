"""Put the CV-30 (grblHAL) on your Wi-Fi over USB.

Asks for the network name and password here (the password is not echoed and
is never written to disk or printed), sends them as $74/$75, sets $73 to
Access Point/Station (keeps the CV30-Laser-Setup access point as a fallback)
or Station only, then restarts the controller and shows its network info.

  python wifi_setup.py [COMx]
"""
import getpass, sys, time

import serial
from serial.tools import list_ports


def find_port():
    ports = [p.device for p in list_ports.comports() if p.vid == 0x303A and p.pid == 0x4001]
    if not ports:
        sys.exit("No CV-30 found on USB (303A:4001).")
    return ports[0]


def send(s, line, wait=1.0):
    s.write((line + "\n").encode())
    end, buf = time.time() + wait, b""
    while time.time() < end:
        buf += s.read(4096)
    return buf.decode("utf-8", "replace").strip()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    with serial.Serial(port, 115200, timeout=0.2) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        info = send(s, "$I")
        if "grblHAL" not in info:
            sys.exit(f"{port} is not running grblHAL (stock firmware?).")
        print(f"grblHAL on {port}.")

        ssid = input("Wi-Fi network name (SSID): ").strip()
        password = getpass.getpass("Wi-Fi password (not shown): ")
        keep_ap = input("Keep the CV30-Laser-Setup access point as a fallback? [Y/n]: ").strip().lower() != "n"

        for line in (f"$74={ssid}", f"$75={password}", f"$73={3 if keep_ap else 1}"):
            reply = send(s, line)
            shown = line if not line.startswith("$75=") else "$75=********"
            print(f"{shown} -> {reply.splitlines()[-1] if reply else 'no reply'}")
        password = None

        print("Restarting the controller ($REBOOT)...")
        send(s, "$REBOOT", 0.5)

    time.sleep(8)
    port = find_port()
    with serial.Serial(port, 115200, timeout=0.2) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        for _ in range(10):
            out = send(s, "$NETIF", 1.5)
            if "IP" in out or "ip" in out:
                break
            time.sleep(2)
        print(out or "No network info yet; try $NETIF again in a moment.")


if __name__ == "__main__":
    main()
