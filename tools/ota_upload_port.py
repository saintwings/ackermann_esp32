# PlatformIO pre-script for the esp32-s3-ota env.
# The IDE's Upload button passes the serial port picked in the status bar
# (e.g. /dev/ttyACM0) as --upload-port to every env, which espota can't use.
# Replace a serial-looking port with the robot's IP (custom_ota_host).
Import("env")

host = env.GetProjectOption("custom_ota_host", "")
port = str(env.get("UPLOAD_PORT") or "")
if host and (not port or port.startswith("/dev/") or port.upper().startswith("COM")):
    if port:
        print(f"[ota] upload port {port} is a serial port, using {host} instead")
    env.Replace(UPLOAD_PORT=host)
