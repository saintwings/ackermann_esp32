#!/usr/bin/env python3

import math
import time 
import argparse
import pygame
import asyncio
import json
import websockets
import os
from motor_controller import ZLAC8015DController, MotorCANController

try:
    import can
except ImportError:
    can = None
    print("⚠️ Warning: 'python-can' module not installed. CAN functions will be disabled.")

try:
    import yaml
except ImportError:
    yaml = None

class DoubleAckermannSteering:
    """Double Ackermann steering controller using m/s units"""
    def __init__(self, wheelbase, track_width, dt, Kp_v, Kd_v, Kp_r, Kd_r):
        self.mode = 0 # mode 0 = normal / 1 = zero turn
        self.L = wheelbase
        self.W = track_width
        self.dt = dt
        self.Kp_v = Kp_v
        self.Kd_v = Kd_v
        self.Kp_r = Kp_r
        self.Kd_r = Kd_r

        self.v_current = 0.0  # m/s
        self.R_current = 1e6  # meters
        self.angle_fl = 0.0
        self.angle_fr = 0.0
        self.angle_rl = 0.0
        self.angle_rr = 0.0

        self.speed_fl = 0.0  # m/s
        self.speed_fr = 0.0  # m/s
        self.speed_rl = 0.0  # m/s
        self.speed_rr = 0.0  # m/s

        self.max_speed = 3.0  # m/s
        self.min_radius = 0.5  # meters
        self.max_radius = 1e6  # meters
        self.max_steering_angle = 30.0  # degrees

    def compute_steering_angle(self, R, side):
        offset = -self.W / 2.0 if side == 'left' else self.W / 2.0
        angle = math.degrees(math.atan((self.L / 2) / (R + offset)))
        if abs(angle) < 0.1:
            angle = 0.0
        # Clamp to max steering angle
        angle = max(-self.max_steering_angle, min(self.max_steering_angle, angle))
        print("angle:", angle)
        return angle

    def compute_wheel_radius(self, side):
        offset = -self.W / 2.0 if side == 'left' else self.W / 2.0
        return math.hypot(self.R_current + offset, self.L / 2)

    def compute_wheel_speed(self, side):
        Rw = self.compute_wheel_radius(side)
        omega = self.v_current / abs(self.R_current)
        return omega * Rw

    def update(self):
        self.angle_fl = self.compute_steering_angle(self.R_current, 'left')
        self.angle_fr = self.compute_steering_angle(self.R_current, 'right')
        self.angle_rl = self.compute_steering_angle(self.R_current, 'right')
        self.angle_rr = self.compute_steering_angle(self.R_current, 'left')


        self.speed_fl = self.compute_wheel_speed('left')
        self.speed_fr = self.compute_wheel_speed('right')
        self.speed_rl = self.compute_wheel_speed('right')
        self.speed_rr = self.compute_wheel_speed('left')

        #self.print_status()

    def print_status(self):
        print(f"v={self.v_current:.3f}")
        print(f"FL={self.speed_fl:.3f} FR={self.speed_fr:.3f}")
        print(f"RL={self.speed_rl:.3f} RR={self.speed_rr:.3f}")
        print(f"R={self.R_current:.3f}")
        print(f"angle_FL={self.angle_fl:.2f}° angle_FR={self.angle_fr:.2f}° ")
        print(f"angle_RL={self.angle_rl:.2f}° angle_RR={self.angle_rr:.2f}°\n")

def read_joystick_buttons(joy):
    pygame.event.pump()

    # Print axes
    for i in range(joy.get_numaxes()):
        print(f"axis {i}: {joy.get_axis(i)}")

    # Print button states
    for i in range(joy.get_numbuttons()):
        state = joy.get_button(i)
        print(f"button {i}: {'pressed' if state else 'released'}")

    # Print hat (D-pad) states if present
    for i in range(joy.get_numhats()):
        print(f"hat {i}: {joy.get_hat(i)}")

def init_mode(mode, can_motor):
    if mode == 0: #normal mode
        print("Enter Mode 0 : normal mode")
        can_motor.send_input_pos(1, 0)
        can_motor.send_input_pos(2, 0)
        time.sleep(2)
    elif mode == 1: #zero turn mode
        print("Enter Mode 1 : zero turn mode")
        can_motor.send_input_pos(1, 0.5)
        can_motor.send_input_pos(2, -1 * 0.5)
        time.sleep(2)

def read_joystick(joy, robot_mode, joy_max_linear_mps=0.6, joy_max_angular_rads=2.0):
    """Read joystick and return linear velocity (m/s) and steering angle (degrees)"""
    linear_mps = 0.0
    angle_deg = 0.0
    mode = robot_mode
    pygame.event.pump()
    if joy.get_button(5):
        if joy.get_button(7) and joy.get_button(3):
            mode = 1 #zero turn mode
        if joy.get_button(7) and joy.get_button(1):
            mode = 0 #normal mode

        axis_forward = -joy.get_axis(3)
        if abs(axis_forward) < 0.05:
            axis_forward = 0.0
        axis_turn = -joy.get_axis(0)
        if abs(axis_turn) < 0.05:
            axis_turn = 0.0
        
        # Scale to m/s and steering angle in degrees
        linear_mps = axis_forward * joy_max_linear_mps
        angle_deg = axis_turn * 30.0  # Max steering angle ±30°

    print(f"[JOY] linear={linear_mps:.3f} m/s, angle={angle_deg:.1f}°")

    return linear_mps, angle_deg, mode


latest_cmd = {"linear": 0.0, "angular": 0.0, "mode": 0}
cmd_lock = asyncio.Lock()
ws_client_count = 0
last_cmd_time = 0.0

def convert_angular_to_steering(linear_mps, angular_rads, wheelbase, max_angle=30.0):
    """Convert linear (m/s) and angular (rad/s) velocity to steering angle (degrees)
    
    Args:
        linear_mps: Linear velocity in m/s
        angular_rads: Angular velocity in rad/s (positive = left turn in standard convention)
        wheelbase: Robot wheelbase in meters
        max_angle: Maximum steering angle limit in degrees (default: 30.0)
    
    Returns:
        Steering angle in degrees (negated to match hardware sign convention)
    """
    if abs(linear_mps) < 0.001:  # Near zero speed
        return 0.0
    
    # Ackermann steering geometry: tan(δ) = (ω * L) / v
    # Negate angular_rads to match hardware sign convention
    tan_angle = (-angular_rads * wheelbase) / linear_mps
    angle_rad = math.atan(tan_angle)
    angle_deg = math.degrees(angle_rad)
    
    # Clamp to max steering angle limit
    return max(-max_angle, min(max_angle, angle_deg))

async def ws_server_handler(websocket):
    """Handle WebSocket connections from controller clients"""
    global latest_cmd, ws_client_count, last_cmd_time
    ws_client_count += 1
    print(f"[WS] Client connected from {websocket.remote_address}")
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                if data.get("type") == "vel_cmd":
                    async with cmd_lock:
                        # New unified protocol: {type, linear, angular}
                        latest_cmd["linear"] = data.get("linear", 0.0)
                        latest_cmd["angular"] = data.get("angular", 0.0)
                        # Mode defaults to 0 (normal), no explicit mode from client
                        latest_cmd["mode"] = 0
                        last_cmd_time = time.time()
                    print(f"[WS] Received: linear={latest_cmd['linear']:.3f} m/s, angular={latest_cmd['angular']:.3f} rad/s")
            except json.JSONDecodeError as e:
                print(f"[WS] JSON parse error: {e}")
    except websockets.exceptions.ConnectionClosed:
        print(f"[WS] Client disconnected")
    finally:
        ws_client_count = max(0, ws_client_count - 1)
        if ws_client_count == 0:
            async with cmd_lock:
                latest_cmd["linear"] = 0.0
                latest_cmd["angular"] = 0.0
                latest_cmd["mode"] = 0

async def start_ws_server(host, port):
    """Start WebSocket server to receive commands"""
    server = await websockets.serve(ws_server_handler, host, port)
    print(f"[WS] Server listening on ws://{host}:{port}")
    return server

def get_network_cmd():
    # Non-async snapshot helper
    return latest_cmd.copy()


def convert_linear_to_mps(value, unit):
    """Convert linear velocity to m/s based on unit specification"""
    unit_norm = (unit or "mps").strip().lower()
    if unit_norm in ("cmps", "cm/s"):
        return value / 100.0
    if unit_norm in ("mps", "m/s"):
        return value
    raise ValueError(f"Unsupported linear unit: {unit}")


def load_robot_drive_config(config_path):
    """Load drivetrain parameters from robot YAML config if available."""
    if not config_path:
        return {}
    if yaml is None:
        print("[CFG] PyYAML not installed; skipping --robot-config")
        return {}
    if not os.path.exists(config_path):
        print(f"[CFG] Config file not found: {config_path}")
        return {}

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        drive = doc.get("drivetrain", {}) if isinstance(doc, dict) else {}
        if not isinstance(drive, dict):
            return {}
        return drive
    except Exception as exc:
        print(f"[CFG] Failed to read config '{config_path}': {exc}")
        return {}

def _retry_forever(max_tries, attempt):
    return max_tries <= 0 or attempt < max_tries

def connect_zlac_pair(port, retry_delay, max_tries, wheel_diameter_m=0.20, gear_ratio=1.0, max_rpm=100.0):
    """Connect to ZLAC motor pair with wheel parameters for physical unit conversion"""
    attempt = 0
    while _retry_forever(max_tries, attempt):
        attempt += 1
        try:
            zlac_1 = ZLAC8015DController(
                port, device_id=1,
                wheel_diameter_m=wheel_diameter_m,
                gear_ratio=gear_ratio,
                max_rpm=max_rpm
            )
            zlac_2 = ZLAC8015DController(
                port, device_id=2,
                wheel_diameter_m=wheel_diameter_m,
                gear_ratio=gear_ratio,
                max_rpm=max_rpm
            )
            zlac_1.set_velocity_mode()
            zlac_2.set_velocity_mode()
            print("[DEV] ZLAC connected with wheel params")
            return zlac_1, zlac_2
        except Exception as exc:
            print(f"[DEV] ZLAC connect failed (attempt {attempt}): {exc}")
            time.sleep(retry_delay)
    raise RuntimeError("ZLAC connect failed: max retries reached")

def connect_can_motor(retry_delay, max_tries):
    attempt = 0
    while _retry_forever(max_tries, attempt):
        attempt += 1
        can_motor = MotorCANController()
        try:
            if not can_motor.setup_bus():
                raise RuntimeError("CAN bus setup failed")
            can_motor.send_set_axis_state(1, 8)
            time.sleep(0.1)
            can_motor.send_set_axis_state(2, 8)
            time.sleep(0.1)
            print("[DEV] CAN motor connected")
            return can_motor
        except Exception as exc:
            print(f"[DEV] CAN connect failed (attempt {attempt}): {exc}")
            try:
                if can_motor.bus:
                    can_motor.bus.shutdown()
            except Exception:
                pass
            time.sleep(retry_delay)
    raise RuntimeError("CAN connect failed: max retries reached")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Double Ackermann control with optional network vel_cmd override")
    parser.add_argument("--ws-port", type=int, default=9100, help="WebSocket server port to listen on (default: 9100)")
    parser.add_argument("--ws-host", default="localhost", help="WebSocket server host to bind to (default: localhost)")
    parser.add_argument("--wheelbase", type=float, default=0.36, help="Wheelbase in meters (default: 0.36)")
    parser.add_argument("--track-width", type=float, default=0.36, help="Track width in meters (default: 0.36)")
    parser.add_argument("--wheel-diameter", type=float, default=0.20, help="Wheel diameter in meters (default: 0.20)")
    parser.add_argument("--max-wheel-rpm", type=float, default=100.0, help="Maximum wheel RPM (default: 100.0)")
    parser.add_argument("--gear-ratio", type=float, default=1.0, help="Motor RPM / wheel RPM (default: 1.0)")
    parser.add_argument("--cmd-linear-unit", default="mps", choices=["cmps", "mps"], help="Unit for incoming vel_cmd linear field")
    parser.add_argument("--joy-max-linear-mps", type=float, default=0.6, help="Joystick full-scale linear speed in m/s")
    parser.add_argument("--joy-max-angular-rads", type=float, default=2.0, help="Joystick full-scale angular speed in rad/s")
    parser.add_argument("--robot-config", default=None, help="Robot YAML config path (optional, reads 'drivetrain' section)")
    parser.add_argument("--reconnect-delay", type=float, default=1.0, help="Delay between device reconnect attempts (seconds)")
    parser.add_argument("--reconnect-max-tries", type=int, default=0, help="Max reconnect tries (0 = infinite)")
    parser.add_argument("--cmd-timeout-sec", type=float, default=0.5, help="Network command timeout in seconds")
    args = parser.parse_args()

    # Load robot config if provided
    drive_cfg = load_robot_drive_config(args.robot_config)
    if drive_cfg:
        args.wheelbase = float(drive_cfg.get("wheelbase_m", args.wheelbase))
        args.track_width = float(drive_cfg.get("track_width_m", args.track_width))
        args.wheel_diameter = float(drive_cfg.get("wheel_diameter_m", args.wheel_diameter))
        args.max_wheel_rpm = float(drive_cfg.get("max_wheel_rpm", args.max_wheel_rpm))
        args.gear_ratio = float(drive_cfg.get("gear_ratio", args.gear_ratio))
        args.cmd_linear_unit = str(drive_cfg.get("linear_cmd_unit", args.cmd_linear_unit))
        args.joy_max_linear_mps = float(drive_cfg.get("joy_max_linear_mps", args.joy_max_linear_mps))
        args.joy_max_angular_rads = float(drive_cfg.get("joy_max_angular_rads", args.joy_max_angular_rads))

    print(
        f"[DRIVE] wheelbase={args.wheelbase:.3f}m track={args.track_width:.3f}m "
        f"wheel_diameter={args.wheel_diameter:.3f}m max_rpm={args.max_wheel_rpm:.1f} "
        f"gear_ratio={args.gear_ratio:.3f} linear_unit={args.cmd_linear_unit}"
    )
    if args.wheel_diameter > 1e-9:
        rpm_per_mps = (60.0 * args.gear_ratio) / (math.pi * args.wheel_diameter)
        print(f"[DRIVE] conversion: 1.00 m/s -> {rpm_per_mps:.2f} rpm")
    else:
        print("[DRIVE] conversion unavailable: wheel_diameter must be > 0")

    pygame.init()
    pygame.joystick.init()
    joy = pygame.joystick.Joystick(0)
    joy.init()

    ### test read joy button ###
    # while(True):
    #     read_joystick_buttons(joy)
    #     time.sleep(0.05)

    ### normal operation ###
    motor_di = [1, -1]
    zlac_1, zlac_2 = connect_zlac_pair(
        '/dev/hubmotor', 
        args.reconnect_delay, 
        args.reconnect_max_tries,
        wheel_diameter_m=args.wheel_diameter,
        gear_ratio=args.gear_ratio,
        max_rpm=args.max_wheel_rpm
    )
    can_motor = connect_can_motor(args.reconnect_delay, args.reconnect_max_tries)

    robot = DoubleAckermannSteering(args.wheelbase, args.track_width, 0.1, Kp_v=4.0, Kd_v=1.5, Kp_r=2.5, Kd_r=0.8)

    async def main_loop():
        # Start WebSocket server
        ws_server = await start_ws_server(args.ws_host, args.ws_port)
        print("ws_host:", args.ws_host)
        print("ws_port:", args.ws_port)
        
        while True:
            now_wall = time.time()
            net = get_network_cmd()
            net_fresh = (now_wall - last_cmd_time) <= max(0.05, args.cmd_timeout_sec)
            net_active = ws_client_count > 0 and net_fresh
            if net_active and net and (abs(net['linear']) > 0.001 or abs(net['angular']) > 0.001):
                # Use network command - convert angular velocity to steering angle
                linear_mps = convert_linear_to_mps(net['linear'], args.cmd_linear_unit)
                angular_rads = net['angular']
                # Convert angular velocity (rad/s) to steering angle (degrees)
                angle_deg = convert_angular_to_steering(linear_mps, angular_rads, robot.L, robot.max_steering_angle)
                robot.mode = net.get('mode', 0)
                print(f"[CTRL] Network cmd: linear={linear_mps:.3f} m/s, angular={angular_rads:.3f} rad/s -> angle={angle_deg:.1f}°")
            else:
                # Use joystick input (returns m/s and angle directly)
                linear_mps, angle_deg, robot.mode = read_joystick(joy, robot.mode, args.joy_max_linear_mps, args.joy_max_angular_rads)

            if robot.mode == 0: # normal mode
                # Use steering angle directly to calculate turn radius
                angle_rad = math.radians(angle_deg)
                robot.v_current = linear_mps
                
                # Calculate turn radius from steering angle using Ackermann geometry
                if abs(angle_rad) > 1e-3:
                    robot.R_current = (robot.L / 2) / math.tan(angle_rad)
                else:
                    robot.R_current = 1e6  # Straight line
                
                robot.update()
                try:
                    # Send wheel speeds directly in m/s
                    zlac_1.set_velocity_mps(motor_di[0] * robot.speed_fl, motor_di[1] * robot.speed_fr)
                    zlac_2.set_velocity_mps(motor_di[0] * robot.speed_rl, motor_di[1] * robot.speed_rr)
                    can_motor.send_input_pos(1, -1 * robot.angle_fl / 45.0)
                    can_motor.send_input_pos(2, -1 * robot.angle_fr / 45.0)
                except Exception as exc:
                    print(f"[DEV] Device error in normal mode: {exc}")
                    zlac_1, zlac_2 = connect_zlac_pair(
                        '/dev/hubmotor', args.reconnect_delay, args.reconnect_max_tries,
                        wheel_diameter_m=args.wheel_diameter, gear_ratio=args.gear_ratio, max_rpm=args.max_wheel_rpm
                    )
                    can_motor = connect_can_motor(args.reconnect_delay, args.reconnect_max_tries)
            elif robot.mode == 1: # zero turn mode
                try:
                    can_motor.send_input_pos(1, 1)
                    can_motor.send_input_pos(2, -1 * 1)
                    # In zero turn mode, use linear velocity for rotation speed
                    zlac_1.set_velocity_mps(motor_di[0] * linear_mps, motor_di[1] * linear_mps * -1)
                    zlac_2.set_velocity_mps(motor_di[0] * linear_mps, motor_di[1] * linear_mps * -1)
                except Exception as exc:
                    print(f"[DEV] Device error in zero turn mode: {exc}")
                    zlac_1, zlac_2 = connect_zlac_pair(
                        '/dev/hubmotor', args.reconnect_delay, args.reconnect_max_tries,
                        wheel_diameter_m=args.wheel_diameter, gear_ratio=args.gear_ratio, max_rpm=args.max_wheel_rpm
                    )
                    can_motor = connect_can_motor(args.reconnect_delay, args.reconnect_max_tries)
            await asyncio.sleep(0.05)

    try:
        asyncio.run(main_loop())
    except KeyboardInterrupt:
        print("Exiting control loop.")
