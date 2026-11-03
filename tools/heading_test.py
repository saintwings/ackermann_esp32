#!/usr/bin/env python3
"""Heading test: log [HDG_S] samples (mag vs fused vs GPS course) and summarise.

  ~/.platformio/penv/bin/python tools/heading_test.py log heading.csv   # record (Ctrl+C to stop)
  ~/.platformio/penv/bin/python tools/heading_test.py report heading.csv

Needs DEBUG_LOG_HEADING 1. Samples only appear while driving forward in NORMAL mode on
RTK fixed (fix=4, every >=0.5 m straight) or RTK float (fix=5, every >=2 m with <10 deg turning;
log-only, does not correct the fused offset), with herr <= HEADING_GPS_MAX_HERR_M.
"""
import csv, glob, math, re, subprocess, sys, time

FIELDS = ["t", "n", "fix", "course", "fused", "mag", "fused_err", "mag_err", "offset", "herr", "v"]
LINE_RE = re.compile(r"\[HDG_S\]\s+(.*)")


def uploading():
    out = subprocess.run(["pgrep", "-af", "esptool|--target upload"], capture_output=True, text=True).stdout
    return bool(out.strip())


def log(path):
    import serial  # only in ~/.platformio/penv/bin/python
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(FIELDS)
        port = None
        while True:
            try:
                if uploading():
                    if port:
                        port.close(); port = None
                        print("upload detected, port released")
                    time.sleep(1); continue
                if port is None:
                    devs = sorted(glob.glob("/dev/ttyACM*"))
                    if not devs:
                        time.sleep(1); continue
                    port = serial.Serial(devs[0], 115200, timeout=0.5)
                    print(f"opened {devs[0]}")
                line = port.readline().decode(errors="replace").strip()
                m = LINE_RE.search(line)
                if not m:
                    continue
                kv = dict(p.split("=", 1) for p in m.group(1).split() if "=" in p)
                w.writerow([f"{time.time():.1f}"] + [kv.get(k, "") for k in FIELDS[1:]])
                f.flush()
                print(line[m.start():])
            except KeyboardInterrupt:
                break
            except (OSError, serial.SerialException) as e:
                print(f"serial: {e}")
                if port:
                    port.close()
                port = None
                time.sleep(1)


def stats(errs):
    n = len(errs)
    mean = sum(errs) / n
    sd = math.sqrt(sum((e - mean) ** 2 for e in errs) / n)
    rms = math.sqrt(sum(e * e for e in errs) / n)
    return mean, sd, rms, max(abs(e) for e in errs)


def report(path):
    rows = [r for r in csv.DictReader(open(path)) if r["course"]]
    if not rows:
        sys.exit("no samples")
    nfix = sum(1 for r in rows if r.get("fix") == "4")
    print(f"{len(rows)} samples: {nfix} RTK fixed, {len(rows) - nfix} RTK float"
          " (errors = GPS course - source, deg)\n")
    print(f"{'source':8} {'mean':>7} {'sd':>6} {'rms':>6} {'max':>6}")
    for k in ("mag_err", "fused_err"):
        mean, sd, rms, mx = stats([float(r[k]) for r in rows])
        print(f"{k[:-4]:8} {mean:7.1f} {sd:6.1f} {rms:6.1f} {mx:6.1f}")
    # fused: skip the first samples while the offset is still converging from the mag seed
    if len(rows) > 10:
        mean, sd, rms, mx = stats([float(r["fused_err"]) for r in rows[10:]])
        print(f"{'fused*':8} {mean:7.1f} {sd:6.1f} {rms:6.1f} {mx:6.1f}   (* after first 10 samples)")

    # Mag error per 45-degree course sector: a sine-shaped pattern is hard-iron
    # distortion (needs calibration); a constant shift is a mounting/declination
    # offset (fix HEADING_OFFSET_DEG by the mean).
    print("\nmag error by GPS course sector")
    print(f"{'sector':>8} {'n':>4} {'mean':>7} {'sd':>6}")
    for s in range(8):
        errs = [float(r["mag_err"]) for r in rows if int(((float(r["course"]) + 22.5) % 360) // 45) == s]
        if errs:
            mean, sd, _, _ = stats(errs)
            print(f"{s * 45:>7}° {len(errs):4} {mean:7.1f} {sd:6.1f}")


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] not in ("log", "report"):
        sys.exit(__doc__)
    (log if sys.argv[1] == "log" else report)(sys.argv[2])
