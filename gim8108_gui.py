#!/usr/bin/env python3
"""
Simple Tkinter GUI for the GIM8108 motor controller.

Reuses MotorController / setup_can_interface / create_bus from GIM8108.py.
All motor calls run on a background worker thread so the GUI never freezes;
their print() output is captured and streamed into the on-screen log.
"""

import io
import queue
import threading
import time
import contextlib
import tkinter as tk
from tkinter import ttk, scrolledtext

import GIM8108 as gm

# methods whose return value should be reflected in the live Motor Info table
INFO_METHODS = {
    "read_versions",
    "read_motor_info",
    "read_status",
    "read_position",
    "read_realtime",
}

# (field key in returned dict, column label)
INFO_FIELDS = [
    ("boot_ver", "Boot Ver"),
    ("app_ver", "App Ver"),
    ("hw_ver", "HW Ver"),
    ("can_proto", "CAN Proto"),
    ("pole_pairs", "Pole Pairs"),
    ("torque_constant", "Torque Const (Nm/A)"),
    ("gear_ratio", "Gear Ratio"),
    ("voltage", "Voltage (V)"),
    ("mode", "Mode"),
    ("fault", "Fault"),
    ("temp", "Temp (C)"),
    ("current", "Current (A)"),
    ("speed", "Speed (RPM)"),
    ("single_deg", "Single Turn (deg)"),
    ("total_deg", "Total Angle (deg)"),
    ("angle_deg", "RT Angle (deg)"),
]


class QueueWriter(io.TextIOBase):
    """File-like object that pushes writes into a thread-safe queue."""

    def __init__(self, q):
        self.q = q

    def write(self, s):
        if s:
            self.q.put(("log", s))
        return len(s)

    def flush(self):
        pass


class MotorGUI:

    def __init__(self, root):
        self.root = root
        root.title("GIM8108 Motor Controller")
        root.geometry("1150x700")
        root.minsize(700, 400)

        self.log_queue = queue.Queue()
        self.cmd_queue = queue.Queue()

        self.bus = None
        self.motors = {}  # slot (1 or 2) -> MotorController

        self.monitor_running = False
        self.monitor_job = None

        self._build_ui()

        # single worker thread executes commands sequentially
        # (avoids concurrent access to the CAN bus)
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self.root.after(100, self._drain_log_queue)

    # ----------------------------------------------------------------
    # UI LAYOUT
    # ----------------------------------------------------------------

    def _build_ui(self):

        # scrollable container so the UI is always reachable regardless
        # of screen size, even if content is taller than the window
        outer = ttk.Frame(self.root)
        outer.pack(fill="both", expand=True)

        canvas = tk.Canvas(outer, highlightthickness=0)
        vscroll = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vscroll.set)
        canvas.pack(side="left", fill="both", expand=True)
        vscroll.pack(side="right", fill="y")

        container = ttk.Frame(canvas)
        container_id = canvas.create_window((0, 0), window=container, anchor="nw")

        def _on_container_configure(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
        container.bind("<Configure>", _on_container_configure)

        def _on_canvas_configure(event):
            canvas.itemconfig(container_id, width=event.width)
        canvas.bind("<Configure>", _on_canvas_configure)

        def _on_mousewheel(event):
            if event.num == 4:
                canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                canvas.yview_scroll(1, "units")
            elif event.delta:
                canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind_all("<Button-4>", _on_mousewheel)
        canvas.bind_all("<Button-5>", _on_mousewheel)
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        pad = {"padx": 4, "pady": 3}

        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=1)

        # ---------------- CAN settings (full width) ----------------
        canf = ttk.LabelFrame(container, text="CAN Settings")
        canf.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)

        ttk.Label(canf, text="Interface:").grid(row=0, column=0, sticky="e", **pad)
        self.iface_var = tk.StringVar(value=gm.CAN_INTERFACE)
        ttk.Entry(canf, textvariable=self.iface_var, width=8).grid(row=0, column=1, **pad)

        ttk.Label(canf, text="Bitrate:").grid(row=0, column=2, sticky="e", **pad)
        self.bitrate_var = tk.StringVar(value=str(gm.CAN_BITRATE))
        ttk.Entry(canf, textvariable=self.bitrate_var, width=10).grid(row=0, column=3, **pad)

        ttk.Label(canf, text="Motor 1 ID:").grid(row=0, column=4, sticky="e", **pad)
        self.motor1_id_var = tk.StringVar(value="0x01")
        ttk.Entry(canf, textvariable=self.motor1_id_var, width=6).grid(row=0, column=5, **pad)

        ttk.Label(canf, text="Motor 2 ID:").grid(row=0, column=6, sticky="e", **pad)
        self.motor2_id_var = tk.StringVar(value="0x02")
        ttk.Entry(canf, textvariable=self.motor2_id_var, width=6).grid(row=0, column=7, **pad)

        # ---------------- connection (full width) ----------------
        conn = ttk.LabelFrame(container, text="Connection")
        conn.grid(row=1, column=0, columnspan=2, sticky="ew", **pad)

        ttk.Button(conn, text="Setup CAN If", command=self.on_setup_can).grid(row=0, column=0, **pad)
        ttk.Button(conn, text="Show CAN Status", command=self.on_show_status).grid(row=0, column=1, **pad)
        ttk.Button(conn, text="Connect Bus", command=self.on_connect_bus).grid(row=0, column=2, **pad)

        self.conn_label = ttk.Label(conn, text="Not connected", foreground="red")
        self.conn_label.grid(row=0, column=3, **pad)

        # ---------------- left / right split ----------------
        left = ttk.Frame(container)
        left.grid(row=2, column=0, sticky="new", **pad)

        right = ttk.Frame(container)
        right.grid(row=2, column=1, sticky="new", **pad)

        # ===== LEFT COLUMN =====

        sel = ttk.LabelFrame(left, text="Target Motor")
        sel.pack(fill="x", **pad)

        self.target_var = tk.StringVar(value="1")
        ttk.Radiobutton(sel, text="Motor 1", variable=self.target_var, value="1").grid(row=0, column=0, **pad)
        ttk.Radiobutton(sel, text="Motor 2", variable=self.target_var, value="2").grid(row=0, column=1, **pad)
        ttk.Radiobutton(sel, text="Both", variable=self.target_var, value="both").grid(row=0, column=2, **pad)

        sysf = ttk.LabelFrame(left, text="System / Info")
        sysf.pack(fill="x", **pad)

        buttons = [
            ("Clear Fault", "clear_fault"),
            ("Disable Motor", "disable_motor"),
            ("Reboot Motor", "reboot_motor"),
            ("Read Versions", "read_versions"),
            ("Read Motor Info", "read_motor_info"),
            ("Read Status", "read_status"),
        ]
        for i, (label, method) in enumerate(buttons):
            ttk.Button(sysf, text=label, command=lambda m=method: self.enqueue_call(m)).grid(
                row=i // 3, column=i % 3, sticky="ew", **pad
            )

        infof = ttk.LabelFrame(left, text="Motor Info (live)")
        infof.pack(fill="x", **pad)

        ttk.Label(infof, text="Field", font=("", 9, "bold")).grid(row=0, column=0, sticky="w", padx=4)
        ttk.Label(infof, text="Motor 1", font=("", 9, "bold")).grid(row=0, column=1, padx=4)
        ttk.Label(infof, text="Motor 2", font=("", 9, "bold")).grid(row=0, column=2, padx=4)

        self.info_vars = {1: {}, 2: {}}
        for i, (key, label) in enumerate(INFO_FIELDS):
            ttk.Label(infof, text=label).grid(row=i + 1, column=0, sticky="w", padx=4)
            for col, mid in ((1, 1), (2, 2)):
                var = tk.StringVar(value="-")
                ttk.Label(infof, textvariable=var, width=13).grid(row=i + 1, column=col, padx=4)
                self.info_vars[mid][key] = var

        # ===== RIGHT COLUMN =====

        rtf = ttk.LabelFrame(right, text="Realtime Data")
        rtf.pack(fill="x", **pad)

        ttk.Button(rtf, text="Read Current", command=lambda: self.enqueue_call("read_current")).grid(row=0, column=0, **pad)
        ttk.Button(rtf, text="Read Speed", command=lambda: self.enqueue_call("read_speed")).grid(row=0, column=1, **pad)
        ttk.Button(rtf, text="Read Position", command=lambda: self.enqueue_call("read_position")).grid(row=0, column=2, **pad)
        ttk.Button(rtf, text="Read Realtime", command=lambda: self.enqueue_call("read_realtime")).grid(row=0, column=3, **pad)

        self.monitor_btn = ttk.Button(rtf, text="Start Monitor", command=self.on_toggle_monitor)
        self.monitor_btn.grid(row=1, column=0, columnspan=2, sticky="ew", **pad)

        ttk.Label(rtf, text="Interval (s):").grid(row=1, column=2, sticky="e")
        self.monitor_interval = tk.StringVar(value="0.5")
        ttk.Entry(rtf, textvariable=self.monitor_interval, width=6).grid(row=1, column=3, sticky="w")

        cfgf = ttk.LabelFrame(right, text="Configuration")
        cfgf.pack(fill="x", **pad)

        ttk.Button(cfgf, text="Set Zero Position", command=lambda: self.enqueue_call("set_current_position_as_zero")).grid(
            row=0, column=0, columnspan=2, sticky="ew", **pad
        )

        self.max_speed_var = self._add_value_row(cfgf, 1, "Max Speed (RPM)", "set_position_max_speed")
        self.max_current_var = self._add_value_row(cfgf, 2, "Max Current (A)", "set_max_current")
        self.accel_var = self._add_value_row(cfgf, 3, "Acceleration (RPM/s)", "set_acceleration")

        ctrlf = ttk.LabelFrame(right, text="Control")
        ctrlf.pack(fill="x", **pad)

        self.torque_var = self._add_value_row(ctrlf, 0, "Torque (A)", "torque_control")
        self.speed_var = self._add_value_row(ctrlf, 1, "Speed (RPM)", "speed_control")
        self.abs_pos_var = self._add_value_row(ctrlf, 2, "Absolute Position (deg)", "absolute_position_control")
        self.rel_pos_var = self._add_value_row(ctrlf, 3, "Relative Position (deg)", "relative_position_control")

        ttk.Button(ctrlf, text="Go Home", command=lambda: self.enqueue_call("go_home")).grid(
            row=4, column=0, columnspan=3, sticky="ew", **pad
        )

        # ---------------- log (full width) ----------------
        logf = ttk.LabelFrame(container, text="Log")
        logf.grid(row=3, column=0, columnspan=2, sticky="ew", **pad)

        self.log_text = scrolledtext.ScrolledText(logf, height=10, width=100, state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

        ttk.Button(logf, text="Clear Log", command=self.clear_log).pack(anchor="e", padx=4, pady=(0, 4))

    def _add_value_row(self, parent, row, label, method):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=4)
        var = tk.StringVar()
        ttk.Entry(parent, textvariable=var, width=10).grid(row=row, column=1, padx=4, pady=4)
        ttk.Button(
            parent, text="Send", command=lambda: self.enqueue_call(method, self._parse_float(var.get()))
        ).grid(row=row, column=2, padx=4, pady=4)
        return var

    # ----------------------------------------------------------------
    # HELPERS
    # ----------------------------------------------------------------

    def _parse_float(self, text):
        try:
            return float(text)
        except ValueError:
            return 0.0

    def _parse_can_id(self, text, default):
        try:
            return int(text.strip(), 0)
        except (ValueError, AttributeError):
            return default

    def _selected_motor_ids(self):
        target = self.target_var.get()
        if target == "both":
            return [1, 2]
        return [int(target)]

    def log(self, text):
        self.log_queue.put(("log", text + "\n"))

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state="disabled")

    def _drain_log_queue(self):
        try:
            while True:
                kind, payload = self.log_queue.get_nowait()
                if kind == "log":
                    self.log_text.configure(state="normal")
                    self.log_text.insert(tk.END, payload)
                    self.log_text.see(tk.END)
                    self.log_text.configure(state="disabled")
                elif kind == "conn":
                    self.conn_label.configure(text=payload[0], foreground=payload[1])
                elif kind == "info":
                    mid, result = payload
                    field_vars = self.info_vars.get(mid, {})
                    for key, value in result.items():
                        if key in field_vars:
                            if isinstance(value, float):
                                field_vars[key].set(f"{value:.3f}")
                            else:
                                field_vars[key].set(str(value))
        except queue.Empty:
            pass
        self.root.after(100, self._drain_log_queue)

    # ----------------------------------------------------------------
    # WORKER THREAD
    # ----------------------------------------------------------------

    def _worker_loop(self):
        while True:
            job = self.cmd_queue.get()
            try:
                job()
            except Exception as e:
                self.log_queue.put(("log", f"ERROR: {e}\n"))

    def _run_captured(self, fn, *args):
        writer = QueueWriter(self.log_queue)
        with contextlib.redirect_stdout(writer):
            return fn(*args)

    def enqueue_call(self, method_name, *args):
        if not self.motors:
            self.log("Not connected. Click 'Connect Bus' first.")
            return

        motor_ids = self._selected_motor_ids()

        def job():
            for mid in motor_ids:
                motor = self.motors.get(mid)
                if motor is None:
                    continue
                fn = getattr(motor, method_name)
                result = self._run_captured(fn, *args)
                if method_name in INFO_METHODS and result:
                    self.log_queue.put(("info", (mid, result)))

        self.cmd_queue.put(job)

    # ----------------------------------------------------------------
    # CONNECTION ACTIONS
    # ----------------------------------------------------------------

    def on_setup_can(self):
        iface = self.iface_var.get().strip() or gm.CAN_INTERFACE
        try:
            bitrate = int(self.bitrate_var.get())
        except ValueError:
            self.log(f"Invalid bitrate '{self.bitrate_var.get()}'")
            return

        def job():
            gm.CAN_INTERFACE = iface
            gm.CAN_BITRATE = bitrate
            self._run_captured(gm.setup_can_interface)
        self.cmd_queue.put(job)

    def on_show_status(self):
        iface = self.iface_var.get().strip() or gm.CAN_INTERFACE

        def job():
            gm.CAN_INTERFACE = iface
            self._run_captured(gm.show_can_status)
        self.cmd_queue.put(job)

    def on_connect_bus(self):
        iface = self.iface_var.get().strip() or gm.CAN_INTERFACE
        motor1_id = self._parse_can_id(self.motor1_id_var.get(), 0x01)
        motor2_id = self._parse_can_id(self.motor2_id_var.get(), 0x02)

        def job():
            try:
                gm.CAN_INTERFACE = iface
                writer = QueueWriter(self.log_queue)
                with contextlib.redirect_stdout(writer):
                    self.bus = gm.create_bus()
                    self.motors = {
                        1: gm.MotorController(bus=self.bus, motor_id=motor1_id),
                        2: gm.MotorController(bus=self.bus, motor_id=motor2_id),
                    }
                self.log_queue.put(("log", f"Motor 1 ID = {hex(motor1_id)}, Motor 2 ID = {hex(motor2_id)}\n"))
                self.log_queue.put(("conn", ("Connected", "green")))
            except Exception as e:
                self.log_queue.put(("log", f"Connection failed: {e}\n"))
                self.log_queue.put(("conn", ("Not connected", "red")))
        self.cmd_queue.put(job)

    # ----------------------------------------------------------------
    # MONITOR
    # ----------------------------------------------------------------

    def on_toggle_monitor(self):
        if self.monitor_running:
            self.monitor_running = False
            self.monitor_btn.configure(text="Start Monitor")
        else:
            if not self.motors:
                self.log("Not connected. Click 'Connect Bus' first.")
                return
            self.monitor_running = True
            self.monitor_btn.configure(text="Stop Monitor")
            self._schedule_monitor_tick()

    def _schedule_monitor_tick(self):
        if not self.monitor_running:
            return

        self.enqueue_call("read_realtime")

        try:
            interval = max(0.1, float(self.monitor_interval.get()))
        except ValueError:
            interval = 0.5

        self.monitor_job = self.root.after(int(interval * 1000), self._schedule_monitor_tick)


def main():
    root = tk.Tk()
    MotorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
