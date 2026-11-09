"""Poll grblHAL status over telnet and print the Pn: (input signals) field
whenever it changes. Read-only.  python watch_pins.py <host> [seconds]"""
import re, socket, sys, time
host = sys.argv[1]; end = time.time() + float(sys.argv[2] if len(sys.argv) > 2 else 120)
s = socket.create_connection((host, 23), timeout=10); s.settimeout(0.2)
last, buf = None, b""
while time.time() < end:
    s.sendall(b"?")
    t = time.time() + 0.25
    while time.time() < t:
        try: buf += s.recv(4096)
        except socket.timeout: pass
    for m in re.finditer(rb"<([^>]*)>", buf):
        fields = m.group(1).decode("utf-8", "replace").split("|")
        pn = next((f[3:] for f in fields if f.startswith("Pn:")), "") or "-"
        state = (fields[0], pn)
        if state != last:
            print(time.strftime("%H:%M:%S"), f"state={state[0]} Pn={state[1]}", flush=True); last = state
    buf = buf[-200:]
s.close()
