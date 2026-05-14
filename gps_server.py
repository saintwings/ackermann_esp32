#!/usr/bin/env python3

import argparse
import asyncio
import os
import struct
import threading
import time
import base64
import socket
import queue
from urllib.parse import urlparse

import pynmea2
import serial
import serial.tools.list_ports
import websockets

# Default serial port configurations by GPS module type
GPS_CONFIGS = {
    "neo_m8u": {
        "port": "/dev/neo_m8u",
        "baudrate": 115200,
        "description": "u-blox NEO-M8U (standard GNSS)"
    },
    "f9p": {
        "port": "/dev/ttyACM0",
        "baudrate": 38400,
        "description": "u-blox ZED-F9P (RTK GNSS with NTRIP)",
        "ntrip_enabled": True
    }
}

# NTRIP Configuration (for F9P RTK)
NTRIP_DEFAULTS = {
    "server": os.getenv("NTRIP_SERVER", "ws://110.78.0.54:2116"),
    "user": os.getenv("NTRIP_USER", "1200100213690"),
    "password": os.getenv("NTRIP_PASS", "EE14"),
    "mountpoint": os.getenv("NTRIP_MOUNTPOINT", "VRS_RTCM32"),
}

DEFAULT_GPS_TYPE = "neo_m8u"

latest_gps = None
lock = threading.Lock()
ws_clients = set()
gps_serial = None  # Global GPS serial object for NTRIP to use
gps_version = None
ntrip_lock = threading.Lock()
ntrip_status = {
    "enabled": False,
    "connected": False,
    "last_rtcm_time": None,
    "last_gga_time": None,
}


def _set_ntrip_status(**updates):
    with ntrip_lock:
        ntrip_status.update(updates)


def _get_ntrip_status():
    with ntrip_lock:
        return dict(ntrip_status)


def _format_age(timestamp):
    if not timestamp:
        return "never"
    return f"{time.time() - timestamp:.1f}s"


def _gps_mode_label(fix_quality: int) -> str:
    status_map = {
        0: "NONE",
        1: "SINGLE",
        2: "DGPS",
        4: "RTK-FIXED",
        5: "RTK-FLOAT",
        6: "DR",
    }
    return status_map.get(fix_quality, "UNKNOWN")


class NTRIPClient:
    """NTRIP client for RTK corrections (F9P only)."""
    
    def __init__(self, hostname: str, port: int, username: str, password: str, mountpoint: str):
        self.hostname = hostname
        self.port = port
        self.username = username
        self.password = password
        self.mountpoint = mountpoint
        self.socket = None
        self.connected = False
        self.gga_queue = queue.Queue(maxsize=10)
    
    async def connect(self):
        """Connect to NTRIP server and send initial request."""
        try:
            # Connect via TCP socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)
            self.socket.connect((self.hostname, self.port))
            
            # Send HTTP GET request with Basic auth
            auth = base64.b64encode(f"{self.username}:{self.password}".encode()).decode()
            mount = self.mountpoint.lstrip("/")
            request = (
                f"GET /{mount} HTTP/1.1\r\n"
                f"Host: {self.hostname}:{self.port}\r\n"
                "Ntrip-Version: Ntrip/2.0\r\n"
                "User-Agent: NTRIP PythonClient/2.0\r\n"
                f"Authorization: Basic {auth}\r\n"
                "Connection: keep-alive\r\n"
                "\r\n"
            )
            self.socket.sendall(request.encode())
            
            # Read response
            response = b""
            while b"\r\n\r\n" not in response and len(response) < 8192:
                chunk = self.socket.recv(1024)
                if not chunk:
                    break
                response += chunk
            response_text = response.decode("utf-8", errors="ignore")
            if "200 OK" in response_text or "ICY 200 OK" in response_text:
                self.connected = True
                _set_ntrip_status(connected=True)
                print("[NTRIP] Connected to NTRIP server")
                return True
            else:
                print(f"[NTRIP] Failed to connect: {response_text[:100]}")
                _set_ntrip_status(connected=False)
                return False
        except Exception as e:
            print(f"[NTRIP] Connection error: {e}")
            _set_ntrip_status(connected=False)
            return False
    
    async def send_gga(self, gga_sentence: str):
        """Send GGA sentence to NTRIP server for position tracking."""
        if not self.connected or not self.socket:
            return False
        try:
            self.socket.sendall((gga_sentence + "\r\n").encode())
            _set_ntrip_status(last_gga_time=time.time())
            return True
        except Exception as e:
            print(f"[NTRIP] Failed to send GGA: {e}")
            self.connected = False
            _set_ntrip_status(connected=False)
            return False
    
    async def receive_rtcm(self) -> bytes:
        """Receive RTCM correction data from server."""
        if not self.connected or not self.socket:
            return b''
        try:
            data = self.socket.recv(4096)
            if not data:
                self.connected = False
                _set_ntrip_status(connected=False)
            else:
                _set_ntrip_status(last_rtcm_time=time.time())
            return data
        except socket.timeout:
            return b''
        except Exception as e:
            print(f"[NTRIP] Receive error: {e}")
            self.connected = False
            _set_ntrip_status(connected=False)
            return b''
    
    def close(self):
        """Close NTRIP connection."""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.connected = False
        _set_ntrip_status(connected=False)

    def queue_gga(self, gga_sentence: str) -> None:
        """Queue latest GGA sentence for the NTRIP thread to send."""
        try:
            self.gga_queue.put_nowait(gga_sentence)
        except queue.Full:
            try:
                _ = self.gga_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self.gga_queue.put_nowait(gga_sentence)
            except queue.Full:
                return


def _parse_ws_url(ws_url: str):
    parsed = urlparse(ws_url if "://" in ws_url else f"ws://{ws_url}")
    host = parsed.hostname or "0.0.0.0"
    port = parsed.port or 9200
    return host, port


def _auto_detect_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        name = (p.device or "")
        if "USB" in name or "ACM" in name or "ttyS" in name or "ttyUSB" in name:
            return name
    return ports[0].device if ports else None


def ntrip_thread_handler(ntrip_client: NTRIPClient):
    """Handle NTRIP corrections in background."""
    print("[NTRIP] Starting NTRIP thread...")
    asyncio.run(_ntrip_main(ntrip_client))


async def _ntrip_main(ntrip_client: NTRIPClient):
    """Main NTRIP connection and correction distribution loop."""
    reconnect_interval = 5.0
    
    while True:
        try:
            if not ntrip_client.connected:
                await ntrip_client.connect()
                if ntrip_client.connected:
                    reconnect_interval = 5.0
                else:
                    await asyncio.sleep(reconnect_interval)
                    reconnect_interval = min(reconnect_interval * 2, 60)
                    continue

            # Send queued GGA sentences (if any)
            while True:
                try:
                    gga_sentence = ntrip_client.gga_queue.get_nowait()
                except queue.Empty:
                    break
                await ntrip_client.send_gga(gga_sentence)
            
            # Receive RTCM corrections and send to GPS device
            rtcm_data = await ntrip_client.receive_rtcm()
            if rtcm_data and gps_serial:
                if rtcm_data.startswith(b"ICY 200") or rtcm_data.startswith(b"HTTP"):
                    await asyncio.sleep(0.01)
                    continue
                try:
                    gps_serial.write(rtcm_data)
                except Exception as e:
                    print(f"[NTRIP] Failed to write RTCM to GPS: {e}")
                    ntrip_client.close()
            
            await asyncio.sleep(0.01)  # Non-blocking small delay
        
        except Exception as e:
            print(f"[NTRIP] Error: {e}")
            ntrip_client.close()
            await asyncio.sleep(reconnect_interval)


def read_gps(serial_port: str, baudrate: int, ntrip_client: NTRIPClient = None):
    """Read NMEA GPS data from serial port."""
    global latest_gps, gps_serial, gps_version
    print("Starting GPS thread...")

    try:
        ser = serial.Serial(serial_port, baudrate, timeout=1)
        gps_serial = ser  # Store for NTRIP to use
        print(f"GPS serial port opened: {serial_port}")
    except Exception as e:
        print(f"Failed to open GPS port: {e}")
        return

    print("-" * 102)
    print(f"{'TIME':<10} | {'NTRIP':<8} | {'RTK STATUS':<10} | {'GPS MODE':<10} | {'SATS':<5} | {'LATITUDE':<12} | {'LONGITUDE':<12} | {'HDOP':<6}")
    print("-" * 102)

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("$GPTXT") or line.startswith("$GNTXT"):
                try:
                    msg = pynmea2.parse(line)
                    text = getattr(msg, "text", None) or getattr(msg, "message", None)
                    if text:
                        with lock:
                            if gps_version is None:
                                gps_version = text
                                print(f"[GPS] Version: {gps_version}")
                except pynmea2.ParseError:
                    continue

            if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                try:
                    msg = pynmea2.parse(line)

                    if not msg.latitude or not msg.longitude:
                        continue

                    gps_data = {
                        "latitude": float(msg.latitude),
                        "longitude": float(msg.longitude),
                        "altitude": float(msg.altitude) if msg.altitude else 0.0,
                        "hdop": float(msg.horizontal_dil) if hasattr(msg, "horizontal_dil") and msg.horizontal_dil else 0.0,
                        "fix_quality": int(msg.gps_qual) if hasattr(msg, "gps_qual") else 0,
                        "satellites": int(getattr(msg, "num_sats", 0) or 0),
                    }

                    with lock:
                        latest_gps = gps_data

                    status = _get_ntrip_status()
                    if status["enabled"]:
                        ntrip_state = "OK" if status["connected"] else "DOWN"
                    else:
                        ntrip_state = "OFF"
                    fix_quality = gps_data["fix_quality"]
                    gps_mode = _gps_mode_label(fix_quality)
                    if fix_quality == 4:
                        rtk_status = "FIXED"
                    elif fix_quality == 5:
                        rtk_status = "FLOAT"
                    else:
                        rtk_status = "NONE"
                    ts = time.strftime("%H:%M:%S")
                    print(
                        f"{ts:<10} | {ntrip_state:<8} | {rtk_status:<10} | {gps_mode:<10} | {gps_data['satellites']:<5} | "
                        f"{gps_data['latitude']:<12.8f} | {gps_data['longitude']:<12.8f} | {gps_data['hdop']:<6.2f}"
                        
                    )
                    
                    # Send GGA to NTRIP for position tracking (if F9P with NTRIP enabled)
                    if ntrip_client and ntrip_client.connected:
                        ntrip_client.queue_gga(line)
                except pynmea2.ParseError:
                    continue
        except Exception:
            continue


def pack_gps_only_binary():
    """Pack GPS data into gps_imu_streamer-compatible binary format with IMU zeros.

    Format: [header(4B)][timestamp(8B)][13 IMU floats][6 GPS floats]
    """
    with lock:
        gps = latest_gps

    if gps is None:
        return None

    header = b"\xAA\xBB\xCC\xDD"
    timestamp = struct.pack("<d", time.time())

    imu_data = struct.pack("<13f", *([0.0] * 13))

    gps_data = struct.pack(
        "<6f",
        float(gps["latitude"]),
        float(gps["longitude"]),
        float(gps.get("altitude") or 0.0),
        float(gps.get("hdop") or 0.0),
        float(gps.get("fix_quality") or 0),
        0.0,
    )

    return header + timestamp + imu_data + gps_data


async def ws_server_handler(websocket):
    ws_clients.add(websocket)
    print(f"[WS] Client connected from {websocket.remote_address}")
    try:
        await websocket.wait_closed()
    finally:
        ws_clients.discard(websocket)
        print("[WS] Client disconnected")


async def ws_heartbeat_loop(interval: float = 5.0):
    """Log heartbeat status to terminal (don't send to clients)."""
    while True:
        await asyncio.sleep(interval)
        with lock:
            gps_ver = gps_version
        status = _get_ntrip_status()
        if status["enabled"]:
            ntrip_state = "connected" if status["connected"] else "disconnected"
            rtcm_age = _format_age(status["last_rtcm_time"])
            gga_age = _format_age(status["last_gga_time"])
            ntrip_info = f"NTRIP {ntrip_state}, RTCM {rtcm_age}, GGA {gga_age}"
        else:
            ntrip_info = "NTRIP disabled"
        gps_info = gps_ver or "unknown"
        if ws_clients:
            print(f"[HEARTBEAT] {len(ws_clients)} client(s) connected | GPS {gps_info} | {ntrip_info}")
        else:
            print(f"[HEARTBEAT] No clients connected | GPS {gps_info} | {ntrip_info}")


async def ws_broadcast_loop():
    broadcast_count = 0
    heartbeat_task = asyncio.create_task(ws_heartbeat_loop(5.0))
    try:
        while True:
            await asyncio.sleep(0.1)
            binary_data = pack_gps_only_binary()
            if not binary_data or not ws_clients:
                continue
            broadcast_count += 1
            if broadcast_count % 10 == 0:
                print(f"[BROADCAST] Sent {broadcast_count} packets to {len(ws_clients)} client(s)")
            dead = []
            for ws in list(ws_clients):
                try:
                    await ws.send(binary_data)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                ws_clients.discard(ws)
    finally:
        heartbeat_task.cancel()


async def ws_server_main(ws_url: str):
    host, port = _parse_ws_url(ws_url)
    server = await websockets.serve(ws_server_handler, host, port)
    print(f"[WS] Server listening on ws://{host}:{port}")
    await ws_broadcast_loop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GPS WebSocket server with multi-module support")
    default_ws = os.environ.get("GPS_WS", "ws://0.0.0.0:9200")
    
    # GPS module type selection
    parser.add_argument(
        "--gps-type",
        choices=list(GPS_CONFIGS.keys()),
        default=DEFAULT_GPS_TYPE,
        help=f"GPS module type. Options: {', '.join(GPS_CONFIGS.keys())} (default: {DEFAULT_GPS_TYPE})"
    )
    
    # WebSocket URL
    parser.add_argument("--ws", default=None, help=f"WebSocket URL (default: {default_ws})")
    
    # Serial port override
    parser.add_argument(
        "--serial-port",
        default=None,
        help="GPS serial port (default: auto-detect or use GPS-type default)"
    )
    
    # Baud rate override
    parser.add_argument("--baudrate", type=int, default=None, help="GPS baud rate (default: use GPS-type default)")
    
    # NTRIP server options (for F9P RTK)
    parser.add_argument(
        "--ntrip-server",
        default=os.environ.get("NTRIP_SERVER", NTRIP_DEFAULTS["server"]),
        help=f"NTRIP server address (default: {NTRIP_DEFAULTS['server']})"
    )
    parser.add_argument(
        "--ntrip-user",
        default=os.environ.get("NTRIP_USER", NTRIP_DEFAULTS["user"]),
        help=f"NTRIP user (default: {NTRIP_DEFAULTS['user']})"
    )
    parser.add_argument(
        "--ntrip-pass",
        default=os.environ.get("NTRIP_PASS", NTRIP_DEFAULTS["password"]),
        help=f"NTRIP password (default: {NTRIP_DEFAULTS['password']})"
    )
    parser.add_argument(
        "--ntrip-mountpoint",
        default=os.environ.get("NTRIP_MOUNTPOINT", NTRIP_DEFAULTS["mountpoint"]),
        help=f"NTRIP mountpoint (default: {NTRIP_DEFAULTS['mountpoint']})"
    )
    
    args = parser.parse_args()

    # Get GPS configuration
    gps_config = GPS_CONFIGS[args.gps_type]
    serial_port = args.serial_port or gps_config["port"]
    baudrate = args.baudrate or gps_config["baudrate"]
    
    # Auto-detect serial port if default doesn't exist
    if not os.path.exists(serial_port):
        detected = _auto_detect_port()
        if detected:
            print(f"Serial port {serial_port} not found. Auto-detected: {detected}")
            serial_port = detected
        else:
            print(f"ERROR: Serial port {serial_port} not found and no USB ports detected")
            exit(1)
    
    print(f"GPS Module: {gps_config['description']}")
    print(f"Serial Port: {serial_port}")
    print(f"Baud Rate: {baudrate}")

    # Initialize NTRIP client if F9P and NTRIP is enabled
    ntrip_client = None
    if args.gps_type == "f9p" and gps_config.get("ntrip_enabled"):
        _set_ntrip_status(enabled=True)
        print("[NTRIP] Configuring RTK corrections for F9P...")
        try:
            # Parse NTRIP server URL (format: ws://host:port or tcp://host:port or host:port)
            ntrip_server = args.ntrip_server
            if ntrip_server.startswith("ws://") or ntrip_server.startswith("tcp://"):
                ntrip_server = ntrip_server.split("://")[1]
            
            if ":" in ntrip_server:
                ntrip_host, ntrip_port = ntrip_server.rsplit(":", 1)
                ntrip_port = int(ntrip_port)
            else:
                ntrip_host = ntrip_server
                ntrip_port = NTRIP_DEFAULTS.get("port", 2116)
            
            ntrip_client = NTRIPClient(
                hostname=ntrip_host,
                port=ntrip_port,
                username=args.ntrip_user,
                password=args.ntrip_pass,
                mountpoint=args.ntrip_mountpoint
            )
            print(f"[NTRIP] Configured for {ntrip_host}:{ntrip_port} (mountpoint: {args.ntrip_mountpoint})")
        except Exception as e:
            print(f"[NTRIP] ERROR: Failed to configure NTRIP: {e}")
            ntrip_client = None
            _set_ntrip_status(enabled=False)
    else:
        _set_ntrip_status(enabled=False)
    
    # Start NTRIP thread if client was initialized
    ntrip_thread = None
    if ntrip_client:
        ntrip_thread = threading.Thread(target=ntrip_thread_handler, args=(ntrip_client,), daemon=True, name="NTRIP")
        ntrip_thread.start()
        print("[NTRIP] NTRIP thread started")

    gps_thread = threading.Thread(target=read_gps, args=(serial_port, baudrate, ntrip_client), daemon=True, name="GPS")
    gps_thread.start()

    ws_url = args.ws or default_ws
    print(f"Starting GPS WebSocket server on {ws_url}")
    asyncio.run(ws_server_main(ws_url))
