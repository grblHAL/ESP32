Import("env")

# Name build artifacts after the active PlatformIO environment.
env.Replace(PROGNAME=env["PIOENV"])
