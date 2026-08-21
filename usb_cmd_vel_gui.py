#!/usr/bin/env python3
"""Simple GUI to send CMD_VEL commands over USB serial to the ESP32 robot."""

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
        self.root.geometry("560x560")
        self.root.minsize(520, 520)

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
        self.func_unit_var = tk.StringVar(value="SEC")
        self.can_ping_id_var = tk.StringVar(value="3")
        self.can_ping_timeout_var = tk.StringVar(value="500")

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

        func = ttk.LabelFrame(self.root, text="FUNC Actuators")
        func.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(func, text="FUNC index").grid(row=0, column=0, sticky="w", **pad)
        ttk.Spinbox(func, textvariable=self.func_index_var, from_=0, to=9, width=6).grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(func, text="Value").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(func, textvariable=self.func_value_var, width=10).grid(row=0, column=3, sticky="w", **pad)

        ttk.Label(func, text="Unit").grid(row=0, column=4, sticky="w", **pad)
        unit_combo = ttk.Combobox(
            func, textvariable=self.func_unit_var, values=["SEC", "DEG", "REL_DEG"], state="readonly", width=10
        )
        unit_combo.grid(row=0, column=5, sticky="w", **pad)

        ttk.Button(func, text="Send FUNC", command=self.send_func).grid(row=1, column=0, columnspan=6, sticky="we", **pad)

        ping = ttk.LabelFrame(self.root, text="CAN_PING (diagnostic)")
        ping.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(ping, text="Node ID").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(ping, textvariable=self.can_ping_id_var, width=8).grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(ping, text="Timeout (ms)").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(ping, textvariable=self.can_ping_timeout_var, width=10).grid(row=0, column=3, sticky="w", **pad)

        ttk.Button(ping, text="Send CAN_PING", command=self.send_can_ping).grid(row=0, column=4, sticky="we", **pad)

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

            if text:
                self.root.after(0, lambda t=text: self._append_log(f"ESP32: {t}"))

        self.root.after(0, self._disconnect_serial)

    def _send_line(self, line: str) -> bool:
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Not connected", "Connect serial first")
            return False
        try:
            self.ser.write((line + "\n").encode("utf-8"))
            self._append_log(f"TX: {line}")
            return True
        except Exception as exc:
            messagebox.showerror("Write failed", str(exc))
            self._append_log(f"Write failed: {exc}")
            return False

    def send_cmd_vel(self) -> None:
        try:
            linear = float(self.linear_var.get().strip())
            angular = float(self.angular_var.get().strip())
            timeout_ms = int(self.timeout_var.get().strip())
        except ValueError:
            messagebox.showerror(
                "Invalid input",
                "Linear and angular must be numbers, timeout must be an integer",
            )
            return

        cmd = f"CMD_VEL {linear:.4f} {angular:.4f} {timeout_ms}"
        self._send_line(cmd)

    def send_mode(self) -> None:
        mode = self.mode_var.get().strip()
        if mode:
            cmd = f"MODE {mode}"
            self._send_line(cmd)

    def send_stop(self) -> None:
        self._send_line("STOP")

    def send_func(self) -> None:
        try:
            index = int(self.func_index_var.get().strip())
            value = float(self.func_value_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid input", "FUNC index must be an integer, value must be a number")
            return

        unit = self.func_unit_var.get().strip()
        if not unit:
            messagebox.showerror("Invalid input", "Select a unit (SEC, DEG, or REL_DEG)")
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
