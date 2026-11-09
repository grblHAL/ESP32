"""Burn one focus-test line: python focus_line.py <host> <y> [power S] [feed]

Moves to X80,<y>, burns a 50 mm line to X130 with M4 at the given power
(default S500 = 50%) and feed (default 600 mm/min), then turns the laser off.
"""
import socket, sys, time

host = sys.argv[1]
y = float(sys.argv[2])
power = int(sys.argv[3]) if len(sys.argv) > 3 else 500
feed = int(sys.argv[4]) if len(sys.argv) > 4 else 600

s = socket.create_connection((host, 23), timeout=5)
s.settimeout(0.3)


def cmd(c, w):
    s.sendall(c.encode() + b"\n")
    out, end = b"", time.time() + w
    while time.time() < end:
        try:
            out += s.recv(4096)
        except socket.timeout:
            pass
    return out.decode(errors="replace").strip()


def wait_idle(t=30):
    st = time.time()
    while time.time() - st < t:
        o = cmd("?", 0.3)
        if "<Alarm" in o or ("<Idle" in o and time.time() - st > 0.8):
            return o
    return o


cmd("", 1)
try:
    cmd("G90 G21", 0.4)
    cmd(f"G0 X80 Y{y} F3000", 0.4)
    wait_idle()
    cmd(f"M4 S{power}", 0.4)
    cmd(f"G1 X130 F{feed}", 0.4)
    cmd("M5", 0.4)
    print(wait_idle().splitlines()[0])
finally:
    cmd("M5", 0.4)
print(f"line at Y{y}: X80-130, S{power}, F{feed}")
