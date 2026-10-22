#!/usr/bin/env python3
"""Simple GUI to send CMD_VEL commands over USB serial to the ESP32 robot."""

import json
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
from serial.tools import list_ports


class UsbCmdVelGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("USB CMD_VEL Sender")
        self.root.geometry("600x820")
        self.root.minsize(560, 700)

        self.ser = None
        self.reader_thread = None
        self.reader_running = False

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.mode_var = tk.StringVar(value="NORMAL")
        self.timeout_var = tk.StringVar(value="250")
        self.linear_var = tk.StringVar(value="0.00")
        self.angular_var = tk.StringVar(value="0.00")
        self.status_var = tk.StringVar(value="Disconnected")
        self.func_index_var = tk.StringVar(value="0")
        self.func_value_var = tk.StringVar(value="5")
        self.func_unit_var = tk.StringVar(value="ON")
        self.can_ping_id_var = tk.StringVar(value="3")
        self.can_ping_timeout_var = tk.StringVar(value="500")

        self.gps_valid_var = tk.StringVar(value="--")
        self.gps_fix_var = tk.StringVar(value="--")
        self.gps_sats_var = tk.StringVar(value="--")
        self.gps_lat_var = tk.StringVar(value="--")
        self.gps_lon_var = tk.StringVar(value="--")
        self.gps_alt_var = tk.StringVar(value="--")
        self.gps_hdop_var = tk.StringVar(value="--")
        self.gps_updated_var = tk.StringVar(value="never")

        # Continuous CMD_VEL: mimics the companion computer, which must resend faster
        # than the command timeout to keep control (mission resumes when it stops).
        self.continuous_var = tk.BooleanVar(value=False)
        self.period_var = tk.StringVar(value="100")
        self.continuous_count = 0
        self.continuous_job = None

        # NAV line from the robot ("NAV ON" enables it, 10 Hz).
        self.nav_enabled = False
        self.nav_vars = {k: tk.StringVar(value="--") for k in (
            "fix", "herr", "heading", "mission", "holding", "override", "wp", "dist_m", "bearing", "speed_cmd", "updated")}

        self._build_ui()
        self._refresh_ports()

    def _build_ui(self) -> None:
        pad = {"padx": 8, "pady": 6}

        top = ttk.LabelFrame(self.root, text="Serial Connection")
        top.pack(fill="x", padx=10, pady=10)

        ttk.Label(top, text="Port").grid(row=0, column=0, sticky="w", **pad)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=22, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="w", **pad)

        ttk.Button(top, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, **pad)

        ttk.Label(top, text="Baud").grid(row=0, column=3, sticky="w", **pad)
        ttk.Entry(top, textvariable=self.baud_var, width=10).grid(row=0, column=4, sticky="w", **pad)

        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=5, **pad)

        mode = ttk.LabelFrame(self.root, text="Robot Mode")
        mode.pack(fill="x", padx=10, pady=(0, 6))

        ttk.Label(mode, text="Mode").grid(row=0, column=0, sticky="w", **pad)
        mode_combo = ttk.Combobox(mode, textvariable=self.mode_var, values=["NORMAL", "ZERO_TURN"], state="readonly", width=15)
        mode_combo.grid(row=0, column=1, sticky="w", **pad)

        ttk.Button(mode, text="Set Mode", command=self.send_mode).grid(row=0, column=2, **pad)

        cmd = ttk.LabelFrame(self.root, text="CMD_VEL")
        cmd.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(cmd, text="Linear (m/s)").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(cmd, textvariable=self.linear_var, width=12).grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(cmd, text="Angular (rad/s)").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(cmd, textvariable=self.angular_var, width=12).grid(row=0, column=3, sticky="w", **pad)

        ttk.Label(cmd, text="Timeout (ms, 0=∞)").grid(row=0, column=4, sticky="w", **pad)
        ttk.Entry(cmd, textvariable=self.timeout_var, width=10).grid(row=0, column=5, sticky="w", **pad)

        ttk.Button(cmd, text="Send CMD_VEL", command=self.send_cmd_vel).grid(row=1, column=0, columnspan=3, sticky="we", **pad)
        ttk.Button(cmd, text="STOP", command=self.send_stop).grid(row=1, column=3, columnspan=3, sticky="we", **pad)

        ttk.Checkbutton(
            cmd, text="Send continuously every", variable=self.continuous_var, command=self._toggle_continuous
        ).grid(row=2, column=0, columnspan=2, sticky="w", **pad)
        ttk.Entry(cmd, textvariable=self.period_var, width=6).grid(row=2, column=2, sticky="w", **pad)
        ttk.Label(cmd, text="ms (keep below timeout)").grid(row=2, column=3, columnspan=3, sticky="w", **pad)

        mission = ttk.LabelFrame(self.root, text="Mission / Companion Computer")
        mission.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Button(mission, text="MISSION_PAUSE", command=lambda: self._send_line("MISSION_PAUSE")).grid(
            row=0, column=0, sticky="we", **pad)
        ttk.Button(mission, text="MISSION_RESUME", command=lambda: self._send_line("MISSION_RESUME")).grid(
            row=0, column=1, sticky="we", **pad)
        self.nav_btn = ttk.Button(mission, text="NAV ON", command=self._toggle_nav)
        self.nav_btn.grid(row=0, column=2, sticky="we", **pad)
        for col in range(3):
            mission.columnconfigure(col, weight=1)

        # Heading source: magnetometer (MAG) or gyro + GPS course correction (GPS).
        hdg = ttk.Frame(mission)
        hdg.grid(row=5, column=0, columnspan=6, sticky="we", padx=4, pady=(4, 2))
        ttk.Button(hdg, text="HEADING MAG", command=lambda: self._send_line("HEADING MAG")).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(hdg, text="HEADING GPS", command=lambda: self._send_line("HEADING GPS")).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(hdg, text="HEADING?", command=lambda: self._send_line("HEADING?")).pack(side="left", expand=True, fill="x", padx=4)

        nav_fields = [
            ("Fix", "fix"), ("GPS error", "herr"), ("Heading", "heading"),
            ("Mission", "mission"), ("Holding", "holding"), ("Override", "override"),
            ("Waypoint", "wp"), ("Distance", "dist_m"), ("Bearing", "bearing"),
            ("Speed cmd", "speed_cmd"), ("Last NAV", "updated"),
        ]
        for i, (label, key) in enumerate(nav_fields):
            r, c = 1 + i // 3, (i % 3) * 2
            ttk.Label(mission, text=label).grid(row=r, column=c, sticky="w", padx=8, pady=2)
            ttk.Label(mission, textvariable=self.nav_vars[key]).grid(row=r, column=c + 1, sticky="w", padx=8, pady=2)

        func = ttk.LabelFrame(self.root, text="FUNC Actuators")
        func.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(func, text="FUNC index").grid(row=0, column=0, sticky="w", **pad)
        ttk.Spinbox(func, textvariable=self.func_index_var, from_=0, to=9, width=6).grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(func, text="Value").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(func, textvariable=self.func_value_var, width=10).grid(row=0, column=3, sticky="w", **pad)

        ttk.Label(func, text="Unit").grid(row=0, column=4, sticky="w", **pad)
        unit_combo = ttk.Combobox(
            func, textvariable=self.func_unit_var, values=["ON", "OFF", "DEG", "REL_DEG"], state="readonly", width=10
        )
        unit_combo.grid(row=0, column=5, sticky="w", **pad)

        ttk.Button(func, text="Send FUNC", command=self.send_func).grid(row=1, column=0, columnspan=6, sticky="we", **pad)

        gpio_test = ttk.LabelFrame(func, text="GPIO Quick Test (FUNC_0 / OUTPUT_A_PIN)")
        gpio_test.grid(row=2, column=0, columnspan=6, sticky="we", padx=8, pady=(0, 6))

        ttk.Button(
            gpio_test, text="FUNC_0 ON", command=lambda: self._send_line("FUNC_0 ON")
        ).grid(row=0, column=0, sticky="we", padx=6, pady=6)
        ttk.Button(
            gpio_test, text="FUNC_0 OFF", command=lambda: self._send_line("FUNC_0 OFF")
        ).grid(row=0, column=1, sticky="we", padx=6, pady=6)
        gpio_test.columnconfigure(0, weight=1)
        gpio_test.columnconfigure(1, weight=1)

        ping = ttk.LabelFrame(self.root, text="CAN_PING (diagnostic)")
        ping.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(ping, text="Node ID").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(ping, textvariable=self.can_ping_id_var, width=8).grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(ping, text="Timeout (ms)").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(ping, textvariable=self.can_ping_timeout_var, width=10).grid(row=0, column=3, sticky="w", **pad)

        ttk.Button(ping, text="Send CAN_PING", command=self.send_can_ping).grid(row=0, column=4, sticky="we", **pad)

        gps = ttk.LabelFrame(self.root, text="GPS (requires GPS_SERIAL_OUTPUT_ENABLE=1 on the robot)")
        gps.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(gps, text="Valid").grid(row=0, column=0, sticky="w", **pad)
        ttk.Label(gps, textvariable=self.gps_valid_var).grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(gps, text="Fix quality").grid(row=0, column=2, sticky="w", **pad)
        ttk.Label(gps, textvariable=self.gps_fix_var).grid(row=0, column=3, sticky="w", **pad)

        ttk.Label(gps, text="Satellites").grid(row=0, column=4, sticky="w", **pad)
        ttk.Label(gps, textvariable=self.gps_sats_var).grid(row=0, column=5, sticky="w", **pad)

        ttk.Label(gps, text="Latitude").grid(row=1, column=0, sticky="w", **pad)
        ttk.Label(gps, textvariable=self.gps_lat_var).grid(row=1, column=1, sticky="w", **pad)

        ttk.Label(gps, text="Longitude").grid(row=1, column=2, sticky="w", **pad)
        ttk.Label(gps, textvariable=self.gps_lon_var).grid(row=1, column=3, sticky="w", **pad)

        ttk.Label(gps, text="Altitude").grid(row=1, column=4, sticky="w", **pad)
        ttk.Label(gps, textvariable=self.gps_alt_var).grid(row=1, column=5, sticky="w", **pad)

        ttk.Label(gps, text="HDOP").grid(row=2, column=0, sticky="w", **pad)
        ttk.Label(gps, textvariable=self.gps_hdop_var).grid(row=2, column=1, sticky="w", **pad)

        ttk.Label(gps, text="Last update").grid(row=2, column=2, sticky="w", **pad)
        ttk.Label(gps, textvariable=self.gps_updated_var).grid(row=2, column=3, columnspan=3, sticky="w", **pad)

        status = ttk.Frame(self.root)
        status.pack(fill="x", padx=10, pady=(0, 6))
        ttk.Label(status, text="Status:").pack(side="left")
        ttk.Label(status, textvariable=self.status_var).pack(side="left", padx=(6, 0))

        log_frame = ttk.LabelFrame(self.root, text="Serial Log")
        log_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.log_text = tk.Text(log_frame, height=10, wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=6, pady=6)
        self.log_text.configure(state="disabled")

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _append_log(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {text}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            if self.port_var.get() not in ports:
                self.port_var.set(ports[0])
        else:
            self.port_var.set("")
        self._append_log("Port list refreshed")

    def _toggle_connection(self) -> None:
        if self.ser and self.ser.is_open:
            self._disconnect_serial()
        else:
            self._connect_serial()

    def _connect_serial(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port", "Select a serial port first")
            return

        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid baud", "Baud must be an integer")
            return

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        except Exception as exc:  # broad except to surface serial errors clearly in GUI
            messagebox.showerror("Connect failed", str(exc))
            self._append_log(f"Connect failed: {exc}")
            self.ser = None
            return

        self.reader_running = True
        self.reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self.reader_thread.start()

        self.connect_btn.configure(text="Disconnect")
        self.status_var.set(f"Connected: {port} @ {baud}")
        self._append_log("Connected")

    def _disconnect_serial(self) -> None:
        self._stop_continuous()
        self.nav_enabled = False
        if hasattr(self, "nav_btn"):
            self.nav_btn.configure(text="NAV ON")
        self.reader_running = False
        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connect_btn.configure(text="Connect")
        self.status_var.set("Disconnected")
        self._append_log("Disconnected")

    def _serial_reader_loop(self) -> None:
        while self.reader_running:
            if not self.ser or not self.ser.is_open:
                break
            try:
                raw = self.ser.readline()
            except Exception as exc:
                self.root.after(0, lambda e=exc: self._append_log(f"Read error: {e}"))
                break

            if not raw:
                continue

            try:
                text = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                text = str(raw)

            if not text:
                continue

            if text.startswith("NAV {"):
                nav = self._parse_nav_line(text)
                if nav is not None:
                    self.root.after(0, lambda n=nav: self._update_nav_display(n))
                continue

            gps_payload = self._parse_gps_line(text)
            if gps_payload is not None:
                self.root.after(0, lambda g=gps_payload: self._update_gps_display(g))
            else:
                self.root.after(0, lambda t=text: self._append_log(f"ESP32: {t}"))

        self.root.after(0, self._disconnect_serial)

    @staticmethod
    def _parse_gps_line(text: str):
        try:
            data = json.loads(text)
        except ValueError:
            return None
        if not isinstance(data, dict) or data.get("type") != "gps":
            return None
        return data.get("gps", {})

    @staticmethod
    def _parse_nav_line(text: str):
        try:
            data = json.loads(text[4:])
        except ValueError:
            return None
        return data if isinstance(data, dict) else None

    def _update_nav_display(self, nav: dict) -> None:
        def fmt(key, spec, suffix=""):
            v = nav.get(key)
            return f"{v:{spec}}{suffix}" if isinstance(v, (int, float)) else "--"

        herr = nav.get("herr")
        self.nav_vars["fix"].set(str(nav.get("fix", "--")))
        self.nav_vars["herr"].set(f"{herr * 100:.1f} cm" if isinstance(herr, (int, float)) and herr >= 0 else "--")
        heading = fmt("heading", ".1f", "\u00b0")
        if "hdg_src" in nav:
            heading += f" ({nav['hdg_src']}, n={nav.get('hdg_n', 0)})"
        self.nav_vars["heading"].set(heading)
        self.nav_vars["mission"].set(str(nav.get("mission", "--")))
        self.nav_vars["holding"].set(str(nav.get("holding", "--")))
        self.nav_vars["override"].set(str(nav.get("override", "--")))
        wp = nav.get("wp", -1)
        self.nav_vars["wp"].set(str(wp) if isinstance(wp, int) and wp >= 0 else "--")
        self.nav_vars["dist_m"].set(fmt("dist_m", ".2f", " m"))
        self.nav_vars["bearing"].set(fmt("bearing", ".1f", "\u00b0"))
        self.nav_vars["speed_cmd"].set(fmt("speed_cmd", ".2f", " m/s"))
        self.nav_vars["updated"].set(time.strftime("%H:%M:%S"))

    def _toggle_nav(self) -> None:
        cmd = "NAV OFF" if self.nav_enabled else "NAV ON"
        if self._send_line(cmd):
            self.nav_enabled = not self.nav_enabled
            self.nav_btn.configure(text="NAV OFF" if self.nav_enabled else "NAV ON")

    def _toggle_continuous(self) -> None:
        if self.continuous_var.get():
            if not self.ser or not self.ser.is_open:
                self.continuous_var.set(False)
                messagebox.showerror("Not connected", "Connect serial first")
                return
            self.continuous_count = 0
            self._append_log("Continuous CMD_VEL started")
            self._continuous_tick()
        else:
            self._stop_continuous()

    def _stop_continuous(self) -> None:
        if self.continuous_job is not None:
            self.root.after_cancel(self.continuous_job)
            self.continuous_job = None
        if self.continuous_count:
            self._append_log(f"Continuous CMD_VEL stopped after {self.continuous_count} commands")
        self.continuous_count = 0
        self.continuous_var.set(False)

    def _continuous_tick(self) -> None:
        self.continuous_job = None
        if not self.continuous_var.get():
            return
        try:
            period_ms = max(20, int(self.period_var.get().strip()))
        except ValueError:
            period_ms = 100
        # Log only the first command; logging every 100 ms would flood the log.
        if not self.send_cmd_vel(log=self.continuous_count == 0):
            self._stop_continuous()
            return
        self.continuous_count += 1
        self.status_var.set(f"Continuous CMD_VEL: {self.continuous_count} sent")
        self.continuous_job = self.root.after(period_ms, self._continuous_tick)

    def _update_gps_display(self, gps: dict) -> None:
        self.gps_valid_var.set(str(gps.get("valid", "--")))
        self.gps_fix_var.set(str(gps.get("fix_quality", "--")))
        self.gps_sats_var.set(str(gps.get("satellites", "--")))
        self.gps_lat_var.set(f"{gps.get('latitude', 0.0):.7f}" if "latitude" in gps else "--")
        self.gps_lon_var.set(f"{gps.get('longitude', 0.0):.7f}" if "longitude" in gps else "--")
        self.gps_alt_var.set(f"{gps.get('altitude', 0.0):.2f}" if "altitude" in gps else "--")
        self.gps_hdop_var.set(f"{gps.get('hdop', 0.0):.2f}" if "hdop" in gps else "--")
        self.gps_updated_var.set(time.strftime("%H:%M:%S"))

    def _send_line(self, line: str, log: bool = True) -> bool:
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Not connected", "Connect serial first")
            return False
        try:
            self.ser.write((line + "\n").encode("utf-8"))
            if log:
                self._append_log(f"TX: {line}")
            return True
        except Exception as exc:
            messagebox.showerror("Write failed", str(exc))
            self._append_log(f"Write failed: {exc}")
            return False

    def send_cmd_vel(self, log: bool = True) -> bool:
        try:
            linear = float(self.linear_var.get().strip())
            angular = float(self.angular_var.get().strip())
            timeout_ms = int(self.timeout_var.get().strip())
        except ValueError:
            messagebox.showerror(
                "Invalid input",
                "Linear and angular must be numbers, timeout must be an integer",
            )
            return False

        cmd = f"CMD_VEL {linear:.4f} {angular:.4f} {timeout_ms}"
        return self._send_line(cmd, log=log)

    def send_mode(self) -> None:
        mode = self.mode_var.get().strip()
        if mode:
            cmd = f"MODE {mode}"
            self._send_line(cmd)

    def send_stop(self) -> None:
        self._stop_continuous()
        self._send_line("STOP")

    def send_func(self) -> None:
        try:
            index = int(self.func_index_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid input", "FUNC index must be an integer")
            return

        unit = self.func_unit_var.get().strip()
        if not unit:
            messagebox.showerror("Invalid input", "Select ON, OFF, DEG, or REL_DEG")
            return

        if unit in ("ON", "OFF"):
            cmd = f"FUNC_{index} {unit}"
            self._send_line(cmd)
            return

        try:
            value = float(self.func_value_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid input", "Value must be a number")
            return

        cmd = f"FUNC_{index} {value:.4f} {unit}"
        self._send_line(cmd)

    def send_can_ping(self) -> None:
        try:
            node_id = int(self.can_ping_id_var.get().strip())
            timeout_ms = int(self.can_ping_timeout_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid input", "Node ID and timeout must be integers")
            return

        cmd = f"CAN_PING {node_id} {timeout_ms}"
        self._send_line(cmd)

    def on_close(self) -> None:
        self.reader_running = False
        self._disconnect_serial()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = UsbCmdVelGui(root)
    app._append_log("GUI ready. Connect and send CMD_VEL commands.")
    root.mainloop()


if __name__ == "__main__":
    main()
