#!/usr/bin/env python3
"""Simple GUI to send CMD_VEL commands over USB serial to the ESP32 robot."""

import json
import re
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
        self.root.geometry("1080x820")
        self.root.minsize(980, 700)

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

        # I/O panel: DS3502 pots, PCF8574 inputs, linear actuator.
        self.pot_count = 3
        self.input_count = 4
        self.pot_vars = [tk.DoubleVar(value=0.0) for _ in range(self.pot_count)]
        self.pot_readback_vars = [tk.StringVar(value="--") for _ in range(self.pot_count)]
        self.input_labels = []
        self.inputs_status_var = tk.StringVar(value="--")
        self.act_target_var = tk.DoubleVar(value=50.0)
        self.act_jog_duty_var = tk.StringVar(value="60")
        self.act_jog_ms_var = tk.StringVar(value="1000")
        self.act_vars = {k: tk.StringVar(value="--") for k in ("pos", "raw", "target", "state", "fault")}
        self.act_pos_value = tk.DoubleVar(value=0.0)
        self.io_live_var = tk.BooleanVar(value=False)
        self.io_period_var = tk.StringVar(value="200")
        self.io_live_job = None
        self.io_live_active = False  # read by the serial thread; Tk vars are main-thread only

        self._build_ui()
        self._refresh_ports()

    def _build_ui(self) -> None:
        pad = {"padx": 8, "pady": 6}

        left = ttk.Frame(self.root)
        left.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(self.root)
        right.pack(side="right", fill="y")
        self._build_io_panel(right)

        top = ttk.LabelFrame(left, text="Serial Connection")
        top.pack(fill="x", padx=10, pady=10)

        ttk.Label(top, text="Port").grid(row=0, column=0, sticky="w", **pad)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=22, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="w", **pad)

        ttk.Button(top, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, **pad)

        ttk.Label(top, text="Baud").grid(row=0, column=3, sticky="w", **pad)
        ttk.Entry(top, textvariable=self.baud_var, width=10).grid(row=0, column=4, sticky="w", **pad)

        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=5, **pad)

        mode = ttk.LabelFrame(left, text="Robot Mode")
        mode.pack(fill="x", padx=10, pady=(0, 6))

        ttk.Label(mode, text="Mode").grid(row=0, column=0, sticky="w", **pad)
        mode_combo = ttk.Combobox(mode, textvariable=self.mode_var, values=["NORMAL", "ZERO_TURN"], state="readonly", width=15)
        mode_combo.grid(row=0, column=1, sticky="w", **pad)

        ttk.Button(mode, text="Set Mode", command=self.send_mode).grid(row=0, column=2, **pad)

        cmd = ttk.LabelFrame(left, text="CMD_VEL")
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

        mission = ttk.LabelFrame(left, text="Mission / Companion Computer")
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

        func = ttk.LabelFrame(left, text="FUNC Actuators")
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

        ping = ttk.LabelFrame(left, text="CAN_PING (diagnostic)")
        ping.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(ping, text="Node ID").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(ping, textvariable=self.can_ping_id_var, width=8).grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(ping, text="Timeout (ms)").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(ping, textvariable=self.can_ping_timeout_var, width=10).grid(row=0, column=3, sticky="w", **pad)

        ttk.Button(ping, text="Send CAN_PING", command=self.send_can_ping).grid(row=0, column=4, sticky="we", **pad)

        gps = ttk.LabelFrame(left, text="GPS (requires GPS_SERIAL_OUTPUT_ENABLE=1 on the robot)")
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

        status = ttk.Frame(left)
        status.pack(fill="x", padx=10, pady=(0, 6))
        ttk.Label(status, text="Status:").pack(side="left")
        ttk.Label(status, textvariable=self.status_var).pack(side="left", padx=(6, 0))

        log_frame = ttk.LabelFrame(left, text="Serial Log")
        log_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.log_text = tk.Text(log_frame, height=10, wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=6, pady=6)
        self.log_text.configure(state="disabled")

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_io_panel(self, parent) -> None:
        pad = {"padx": 8, "pady": 4}

        pots = ttk.LabelFrame(parent, text="DS3502 Pots (POT <ch> <0-100>)")
        pots.pack(fill="x", padx=10, pady=10)
        for i in range(self.pot_count):
            ch = i + 1
            ttk.Label(pots, text=f"POT {ch}").grid(row=i, column=0, sticky="w", **pad)
            scale = ttk.Scale(pots, from_=0, to=100, variable=self.pot_vars[i], length=200)
            scale.grid(row=i, column=1, sticky="we", **pad)
            # Send on release only — dragging would flood the serial link.
            scale.bind("<ButtonRelease-1>", lambda _e, c=ch: self.send_pot(c))
            ttk.Spinbox(pots, from_=0, to=100, increment=1, textvariable=self.pot_vars[i], width=5,
                        command=lambda c=ch: self.send_pot(c)).grid(row=i, column=2, **pad)
            ttk.Button(pots, text="Set", width=5, command=lambda c=ch: self.send_pot(c)).grid(row=i, column=3, **pad)
            ttk.Label(pots, textvariable=self.pot_readback_vars[i], width=14).grid(row=i, column=4, sticky="w", **pad)
        ttk.Button(pots, text="POT? (read back)", command=lambda: self._send_line("POT?")).grid(
            row=self.pot_count, column=0, columnspan=5, sticky="we", **pad)
        pots.columnconfigure(1, weight=1)

        inputs = ttk.LabelFrame(parent, text="Digital Inputs (PCF8574)")
        inputs.pack(fill="x", padx=10, pady=(0, 10))
        row = ttk.Frame(inputs)
        row.pack(fill="x", padx=6, pady=6)
        for i in range(self.input_count):
            lbl = tk.Label(row, text=f"IN {i + 1}", width=8, relief="groove", bg="#d0d0d0", pady=6)
            lbl.pack(side="left", expand=True, fill="x", padx=4)
            self.input_labels.append(lbl)
        status = ttk.Frame(inputs)
        status.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Button(status, text="IN?", command=lambda: self._send_line("IN?")).pack(side="left")
        ttk.Label(status, textvariable=self.inputs_status_var).pack(side="left", padx=8)

        act = ttk.LabelFrame(parent, text="Linear Actuator (VNH2SP30)")
        act.pack(fill="x", padx=10, pady=(0, 10))
        ttk.Label(act, text="Position").grid(row=0, column=0, sticky="w", **pad)
        ttk.Progressbar(act, variable=self.act_pos_value, maximum=100).grid(
            row=0, column=1, columnspan=3, sticky="we", **pad)
        ttk.Label(act, textvariable=self.act_vars["pos"], width=8).grid(row=0, column=4, sticky="w", **pad)

        fields = [("Target", "target"), ("Raw ADC", "raw"), ("State", "state"), ("Fault", "fault")]
        for i, (label, key) in enumerate(fields):
            r, c = 1 + i // 2, (i % 2) * 2
            ttk.Label(act, text=label).grid(row=r, column=c, sticky="w", **pad)
            ttk.Label(act, textvariable=self.act_vars[key]).grid(row=r, column=c + 1, sticky="w", **pad)

        ttk.Label(act, text="Move to %").grid(row=3, column=0, sticky="w", **pad)
        ttk.Scale(act, from_=0, to=100, variable=self.act_target_var).grid(
            row=3, column=1, columnspan=2, sticky="we", **pad)
        ttk.Spinbox(act, from_=0, to=100, increment=1, textvariable=self.act_target_var, width=5).grid(
            row=3, column=3, **pad)
        ttk.Button(act, text="Move", command=self.send_act_move).grid(row=3, column=4, sticky="we", **pad)

        ttk.Label(act, text="Jog duty %").grid(row=4, column=0, sticky="w", **pad)
        ttk.Entry(act, textvariable=self.act_jog_duty_var, width=6).grid(row=4, column=1, sticky="w", **pad)
        ttk.Label(act, text="ms").grid(row=4, column=2, sticky="e", **pad)
        ttk.Entry(act, textvariable=self.act_jog_ms_var, width=6).grid(row=4, column=3, sticky="w", **pad)
        jog = ttk.Frame(act)
        jog.grid(row=5, column=0, columnspan=5, sticky="we", padx=4, pady=2)
        ttk.Button(jog, text="\u25c0 Retract", command=lambda: self.send_act_jog(-1)).pack(
            side="left", expand=True, fill="x", padx=4)
        ttk.Button(jog, text="Extend \u25b6", command=lambda: self.send_act_jog(1)).pack(
            side="left", expand=True, fill="x", padx=4)
        ttk.Button(jog, text="ACT?", command=lambda: self._send_line("ACT?")).pack(
            side="left", expand=True, fill="x", padx=4)

        ttk.Button(act, text="ACT STOP", command=lambda: self._send_line("ACT STOP")).grid(
            row=6, column=0, columnspan=5, sticky="we", **pad)
        act.columnconfigure(1, weight=1)

        live = ttk.LabelFrame(parent, text="Live I/O")
        live.pack(fill="x", padx=10, pady=(0, 10))
        ttk.Checkbutton(live, text="Poll IN? / ACT? every", variable=self.io_live_var,
                        command=self._toggle_io_live).pack(side="left", padx=8, pady=6)
        ttk.Entry(live, textvariable=self.io_period_var, width=6).pack(side="left", pady=6)
        ttk.Label(live, text="ms").pack(side="left", padx=4, pady=6)

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
        self._stop_io_live()
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

            if self._handle_io_line(text):
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

    # ── I/O panel ────────────────────────────────────────────────────────────
    _RE_DISABLED = re.compile(r"\[USB_SERIAL\] (POT|IN|ACT) rejected: (\w+_ENABLE) is 0")
    _RE_IN = re.compile(r"\[USB_SERIAL\] IN((?: \d+=[01])+) raw=(0x[0-9A-Fa-f]+)")
    _RE_IN_ERR = re.compile(r"\[USB_SERIAL\] IN: (.*)")
    _RE_ACT = re.compile(
        r"\[USB_SERIAL\] ACT pos=(-?[\d.]+)% raw=(\d+) target=(-?[\d.]+)% state=(\w+) fault=(\w+)")
    _RE_POT = re.compile(r"\[USB_SERIAL\] POT (\d+)(?: \(0x[0-9A-Fa-f]+\):| set) (?:wiper=(\d+)/\d+ \(([\d.]+)%\)|(.*))")

    def _handle_io_line(self, text: str) -> bool:
        """Updates the I/O panel from a status line. Returns True when the line
        should not be logged (polled status while Live I/O is on)."""
        live = self.io_live_active
        m = self._RE_DISABLED.match(text)
        if m:
            self.root.after(0, lambda mod=m.group(1), flag=m.group(2): self._show_module_disabled(mod, flag))
            return live
        m = self._RE_IN.match(text)
        if m:
            states = {int(k): v == "1" for k, v in (p.split("=") for p in m.group(1).split())}
            raw = m.group(2)
            self.root.after(0, lambda s=states, r=raw: self._update_inputs(s, f"OK  raw={r}"))
            return live
        m = self._RE_IN_ERR.match(text)
        if m:
            msg = m.group(1)
            self.root.after(0, lambda t=msg: self._update_inputs({}, t))
            return live
        m = self._RE_ACT.match(text)
        if m:
            self.root.after(0, lambda g=m.groups(): self._update_actuator(*g))
            return live
        m = self._RE_POT.match(text)
        if m:
            ch = int(m.group(1))
            if 1 <= ch <= self.pot_count:
                if m.group(2) is not None:
                    shown = f"{int(m.group(2))}/127 ({float(m.group(3)):.0f}%)"
                else:
                    shown = m.group(4)
                self.root.after(0, lambda c=ch, v=shown: self.pot_readback_vars[c - 1].set(v))
        return False

    def _show_module_disabled(self, module: str, flag: str) -> None:
        text = f"disabled ({flag}=0)"
        if module == "POT":
            for v in self.pot_readback_vars:
                v.set("disabled")
        elif module == "IN":
            self._update_inputs({}, text)
        elif module == "ACT":
            self.act_pos_value.set(0.0)
            for key in ("pos", "raw", "target", "fault"):
                self.act_vars[key].set("--")
            self.act_vars["state"].set(text)

    def _update_inputs(self, states: dict, status: str) -> None:
        for i, lbl in enumerate(self.input_labels):
            on = states.get(i + 1)
            if on is None:
                lbl.configure(bg="#d0d0d0", fg="black")
            else:
                lbl.configure(bg="#3cb043" if on else "#f0f0f0", fg="white" if on else "black")
        self.inputs_status_var.set(f"{status}  ({time.strftime('%H:%M:%S')})")

    def _update_actuator(self, pos, raw, target, state, fault) -> None:
        pos_f = float(pos)
        self.act_pos_value.set(max(0.0, min(100.0, pos_f)))
        self.act_vars["pos"].set(f"{pos_f:.1f}%")
        self.act_vars["raw"].set(raw)
        self.act_vars["target"].set(f"{float(target):.1f}%")
        self.act_vars["state"].set(state)
        self.act_vars["fault"].set(fault)

    def send_pot(self, ch: int) -> None:
        try:
            pct = max(0.0, min(100.0, float(self.pot_vars[ch - 1].get())))
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid input", "Pot value must be 0-100")
            return
        self.pot_vars[ch - 1].set(round(pct))
        self._send_line(f"POT {ch} {pct:.0f}")

    def send_act_move(self) -> None:
        try:
            pct = max(0.0, min(100.0, float(self.act_target_var.get())))
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid input", "Target must be 0-100")
            return
        self.act_target_var.set(round(pct, 1))
        self._send_line(f"ACT {pct:.1f}")

    def send_act_jog(self, direction: int) -> None:
        try:
            duty = abs(float(self.act_jog_duty_var.get().strip()))
            ms = int(self.act_jog_ms_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid input", "Jog duty must be a number, ms an integer")
            return
        if duty > 100 or not 1 <= ms <= 5000:
            messagebox.showerror("Invalid input", "Jog duty 0-100 %, time 1-5000 ms")
            return
        self._send_line(f"ACT JOG {direction * duty:.0f} {ms}")

    def _toggle_io_live(self) -> None:
        if self.io_live_var.get():
            if not self.ser or not self.ser.is_open:
                self.io_live_var.set(False)
                messagebox.showerror("Not connected", "Connect serial first")
                return
            self.io_live_active = True
            self._append_log("Live I/O polling started")
            self._io_live_tick()
        else:
            self._stop_io_live()

    def _stop_io_live(self) -> None:
        self.io_live_active = False
        if self.io_live_job is not None:
            self.root.after_cancel(self.io_live_job)
            self.io_live_job = None
            self._append_log("Live I/O polling stopped")
        self.io_live_var.set(False)

    def _io_live_tick(self) -> None:
        self.io_live_job = None
        if not self.io_live_var.get() or not self.ser or not self.ser.is_open:
            return
        try:
            period_ms = max(100, int(self.io_period_var.get().strip()))
        except ValueError:
            period_ms = 200
        if not (self._send_line("IN?", log=False) and self._send_line("ACT?", log=False)):
            self._stop_io_live()
            return
        self.io_live_job = self.root.after(period_ms, self._io_live_tick)

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
