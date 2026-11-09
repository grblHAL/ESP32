"""Wireless update for the CV-30 running grblHAL.

Uploads a CV-30-grblhal-N.bin to the box's microSD card over FTP, checks the
size, sends $UPDATE=<file> over telnet (the box hands it to the stock
installer, which installs it and boots it), waits for the box to come back
and checks that $I reports "build N".

  python ota.py <image.bin> [host]       (host defaults to cv30-laser.local)
"""
import ftplib, os, re, socket, sys, time

image = sys.argv[1]
host = sys.argv[2] if len(sys.argv) > 2 else "cv30-laser.local"
name = os.path.basename(image)
if not re.fullmatch(r".*CV-30.*\.bin", name):
    sys.exit("The stock installer only takes .bin files with CV-30 in the name.")
build = re.search(r"grblhal-(\w+)\.bin$", name)


def telnet(cmd, wait=3.0):
    with socket.create_connection((host, 23), timeout=10) as s:
        s.settimeout(0.3)
        s.sendall(b"?")
        out, end = b"", time.time() + 1
        while time.time() < end:
            try:
                out += s.recv(4096)
            except socket.timeout:
                pass
        s.sendall(cmd.encode() + b"\n")
        out, end = b"", time.time() + wait
        while time.time() < end:
            try:
                d = s.recv(4096)
                if not d:
                    break
                out += d
            except socket.timeout:
                pass
    return out.decode("utf-8", "replace").strip()


print(f"Uploading {name} to {host} ...")
ftp = ftplib.FTP()
ftp.connect(host, 21, timeout=30)
ftp.login()
t = time.time()
with open(image, "rb") as f:
    ftp.storbinary(f"STOR {name}", f, 4096)
listing = []
ftp.retrlines("LIST /", listing.append)
ftp.quit()
size = os.path.getsize(image)
remote = next((int(l.split()[4]) for l in listing if l.split()[-1] == name), None)
print(f"  {size} bytes in {time.time() - t:.0f} s; on the card: {remote}")
if remote != size:
    sys.exit("Size on the card does not match; not installing.")

print(telnet(f"$UPDATE={name}"))

print("Waiting for the box to install it and come back ...")
time.sleep(15)
for _ in range(40):
    try:
        info = telnet("$I", 2)
        board = next((l for l in info.splitlines() if l.startswith("[BOARD:")), "")
        if board:
            print(board)
            if build and f"build {build.group(1)})" in board:
                print(f"OK: build {build.group(1)} is running.")
            else:
                print("WARNING: the running build is not the one uploaded.")
            break
    except OSError:
        pass
    time.sleep(3)
else:
    print("The box did not come back on the network within ~2 minutes (USB/power-cycle escape still work).")
