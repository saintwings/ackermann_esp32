import struct
import time
import subprocess
from datetime import datetime
import minimalmodbus

try:
    import can
except ImportError:
    can = None
    print(" Warning: 'python-can' module not installed. CAN functions will be disabled.")


class MotorCANController:
    def __init__(self, channel='can0', bitrate=500000, motor_positions=None, log_file='can_motor_log.txt', interface='socketcan'):
        """
        Initialize Motor CAN Controller for ODrive/Steadywin motors
        
        Supports Steadywin motors with ODrive CAN protocol.
        
        Args:
            channel: CAN channel
                - For socketcan: 'can0', 'can1', etc.
                - For CANABLE: '/dev/ttyACM0', 'COM3' (Windows), etc.
            bitrate: CAN bus speed (communication between CAN devices)
                - Steadywin motors typically use 500000 (500kbps)
                - Common values: 125000, 250000, 500000, 1000000
            motor_positions: Dictionary mapping motor node IDs to position multipliers
                - Example: {0: 1.0, 1: -1.0} for motors with IDs 0 and 1
            log_file: Path to log file
            interface: CAN interface type
                - 'socketcan': Native Linux CAN (requires 'ip link' setup)
                - 'slcan': Serial Line CAN for CANABLE 2.0 devices
        
        Example usage:
            # Steadywin motors with CANABLE 2.0
            motor = MotorCANController(
                channel='/dev/ttyACM0',
                bitrate=500000,
                interface='slcan',
                motor_positions={0: 1.0, 1: -1.0}
            )
            
            # Steadywin motors with SocketCAN
            motor = MotorCANController(
                channel='can0',
                bitrate=500000,
                interface='socketcan',
                motor_positions={0: 1.0, 1: -1.0}
            )
        """
        self.channel = channel
        self.bitrate = bitrate
        self.motor_positions = motor_positions or {0: 1.0, 1: -1.0}  # Steadywin default node IDs
        self.log_file = log_file
        self.interface = interface
        self.bus = None

    def float_to_bytes_little(self, f):
        return list(struct.pack('<f', f))

    def uint16_to_bytes_little(self, u):
        return list(struct.pack('<H', u))
    
    def bytes_to_float_little(self, data):
        """Convert 4 bytes (little-endian) to float"""
        return struct.unpack('<f', bytes(data[:4]))[0]
    
    def bytes_to_int32_little(self, data):
        """Convert 4 bytes (little-endian) to signed int32"""
        return struct.unpack('<i', bytes(data[:4]))[0]
    
    def bytes_to_uint16_little(self, data):
        """Convert 2 bytes (little-endian) to unsigned int16"""
        return struct.unpack('<H', bytes(data[:2]))[0]

    def log(self, msg):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.log_file, 'a') as f:
            f.write(f"[{timestamp}] {msg}\n")
        print(f"[{timestamp}] {msg}")

    def is_can0_up(self):
        result = subprocess.run(['ip', 'link', 'show', self.channel], capture_output=True, text=True)
        return 'state UP' in result.stdout

    def bring_up_can0(self):
        self.log("Bringing up CAN interface...")
        subprocess.run(['sudo', 'ip', 'link', 'set', self.channel, 'down'], stdout=subprocess.DEVNULL)
        subprocess.run(['sudo', 'ip', 'link', 'set', self.channel, 'type', 'can', 'bitrate', str(self.bitrate)], stdout=subprocess.DEVNULL)
        subprocess.run(['sudo', 'ip', 'link', 'set', self.channel, 'up'], stdout=subprocess.DEVNULL)
        time.sleep(0.2)
        self.log(" CAN interface is UP.")

    def setup_bus(self):
        if not can:
            self.log(" python-can not available")
            return False
        
        # For socketcan, ensure interface is up
        if self.interface == 'socketcan':
            if not self.is_can0_up():
                self.bring_up_can0()
        elif self.interface == 'slcan':
            # CANABLE 2.0 uses slcan interface, no need to bring up interface
            self.log(f"Using CANABLE 2.0 on {self.channel} with slcan interface")
        
        try:
            if self.interface == 'slcan':
                # For CANABLE 2.0, use slcan interface
                self.log("Opening CANABLE 2.0 connection...")
                self.bus = can.interface.Bus(
                    interface='slcan',
                    channel=self.channel,
                    tty_baudrate=115200,  # Serial baudrate for USB connection
                    bitrate=self.bitrate,  # CAN bus bitrate
                    sleep_after_open=0.5,  # Reduced wait time
                    timeout=0.1
                )
                self.log(f"CAN bus opened on {self.channel} using slcan (bitrate: {self.bitrate}bps)")
            else:
                # For socketcan (default)
                self.bus = can.interface.Bus(interface='socketcan', channel=self.channel, bitrate=self.bitrate)
                self.log(f"CAN bus opened on {self.channel} using socketcan (bitrate: {self.bitrate}bps)")
            
            return True
        except Exception as e:
            self.log(f" Failed to open CAN bus: {e}")
            return False

    def send_set_axis_state(self, node_id, state):
        can_id = (node_id << 5) | 0x07
        data = [state] + [0x00] * 7
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        self.log(f"[ID={node_id}] Set Axis State = {state}")

    def send_input_pos(self, node_id, position_turns, vel_ff=0.01, torque_ff=0):
        # Position command wrapper
        self.send_command(node_id, mode='position', value=position_turns, vel_ff=vel_ff, torque_ff=torque_ff)

    def send_command(self, node_id, mode='position', value=0.0, vel_ff=0.0, torque_ff=0.0):
        if mode == 'position':
            cmd_id = 0x0C
            can_id = (node_id << 5) | cmd_id
            data = self.float_to_bytes_little(value) + \
                   self.uint16_to_bytes_little(int(vel_ff * 1000)) + \
                   self.uint16_to_bytes_little(int(torque_ff * 1000))
            label = f"Position = {value:.2f}, Vel_FF = {vel_ff:.2f}, Torque_FF = {torque_ff:.2f}"

        elif mode == 'velocity':
            cmd_id = 0x0D
            can_id = (node_id << 5) | cmd_id
            data = self.float_to_bytes_little(value) + [0x00] * 4
            label = f"Velocity = {value:.2f}"

        elif mode == 'torque':
            cmd_id = 0x0E
            can_id = (node_id << 5) | cmd_id
            data = self.float_to_bytes_little(value) + [0x00] * 4
            label = f"Torque = {value:.2f}"

        else:
            self.log(f" Unsupported mode: {mode}")
            return

        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        # self.log(f"[ID={node_id}] {mode.capitalize()} Command → {label}")

    def receive_message(self, timeout=0.1):
        """
        Receive a single CAN message from the bus
        
        Args:
            timeout: Time to wait for a message in seconds
            
        Returns:
            can.Message object or None if timeout
        """
        try:
            msg = self.bus.recv(timeout=timeout)
            return msg
        except Exception as e:
            self.log(f"Error receiving message: {e}")
            return None
    
    def parse_encoder_estimates(self, msg):
        """
        Parse encoder position and velocity feedback (CMD_ID 0x09)
        
        Returns:
            dict with 'node_id', 'position', 'velocity'
        """
        node_id = (msg.arbitration_id >> 5) & 0x3F
        cmd_id = msg.arbitration_id & 0x1F
        
        if cmd_id == 0x09 and len(msg.data) == 8:
            position = self.bytes_to_float_little(msg.data[0:4])
            velocity = self.bytes_to_float_little(msg.data[4:8])
            return {
                'node_id': node_id,
                'position': position,
                'velocity': velocity,
                'type': 'encoder_estimates'
            }
        return None
    
    def parse_heartbeat(self, msg):
        """
        Parse heartbeat message (CMD_ID 0x01)
        
        Returns:
            dict with 'node_id', 'error', 'state', 'result', 'traj_done'
        """
        node_id = (msg.arbitration_id >> 5) & 0x3F
        cmd_id = msg.arbitration_id & 0x1F
        
        if cmd_id == 0x01 and len(msg.data) >= 8:
            error = struct.unpack('<I', bytes(msg.data[0:4]))[0]
            state = msg.data[4]
            result = msg.data[5]
            traj_done = msg.data[7]
            return {
                'node_id': node_id,
                'error': error,
                'state': state,
                'result': result,
                'traj_done': traj_done,
                'type': 'heartbeat'
            }
        return None
    
    def get_encoder_estimates(self, node_id, timeout=0.5):
        """
        Request and get encoder estimates from a motor
        
        Args:
            node_id: Motor node ID
            timeout: Time to wait for response
            
        Returns:
            dict with position and velocity, or None
        """
        # Send Get_Encoder_Estimates command (CMD_ID 0x09)
        can_id = (node_id << 5) | 0x09
        msg = can.Message(arbitration_id=can_id, data=[0x00]*8, is_extended_id=False)
        self.bus.send(msg)
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            response = self.receive_message(timeout=0.1)
            if response:
                parsed = self.parse_encoder_estimates(response)
                if parsed and parsed['node_id'] == node_id:
                    return parsed
        
        return None
    
    def read_messages(self, duration=1.0, parse=True):
        """
        Read all CAN messages for a specified duration
        
        Args:
            duration: Time to read messages in seconds
            parse: Whether to parse known message types
            
        Returns:
            List of messages (parsed if parse=True, raw otherwise)
        """
        messages = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            msg = self.receive_message(timeout=0.1)
            if msg:
                if parse:
                    # Try to parse known message types
                    parsed = self.parse_encoder_estimates(msg) or self.parse_heartbeat(msg)
                    if parsed:
                        messages.append(parsed)
                        self.log(f"[ID={parsed['node_id']}] {parsed['type']}: {parsed}")
                    else:
                        messages.append({'raw': msg, 'type': 'unknown'})
                else:
                    messages.append(msg)
        
        return messages

    def demo_sequence(self):
        if not self.setup_bus():
            return
        try:
            # Step 1: Enable motors
            for node_id in self.motor_positions:
                self.send_set_axis_state(node_id, 8)
                time.sleep(0.1)

            # Step 2: Send position sequence
            sequence = [(0, 0), (1, -1), (0, 0), (-1, 1), (0, 0)]
            for left, right in sequence:
                self.send_input_pos(0, left)  # Motor node ID 0
                self.send_input_pos(1, right)  # Motor node ID 1
                time.sleep(3)

            # Step 3: Stop motors
            for node_id in self.motor_positions:
                self.send_set_axis_state(node_id, 1)
                time.sleep(0.1)

        except Exception as e:
            self.log(f" Error: {e}")
        finally:
            try:
                self.bus.shutdown()
                self.log("CAN bus shut down cleanly.")
            except Exception:
                pass
    
    def demo_with_feedback(self):
        """Demo sequence with motor feedback reading"""
        if not self.setup_bus():
            return
        try:
            # Step 1: Enable motors and get initial state
            self.log("\n=== Enabling Motors ===")
            for node_id in self.motor_positions:
                self.send_set_axis_state(node_id, 8)
                time.sleep(0.2)
                # Get encoder feedback
                feedback = self.get_encoder_estimates(node_id)
                if feedback:
                    self.log(f"[ID={node_id}] Initial - Pos: {feedback['position']:.3f}, Vel: {feedback['velocity']:.3f}")

            # Step 2: Send commands and monitor feedback
            self.log("\n=== Running Movement Sequence ===")
            sequence = [(0, 0), (4, -4), (0, 0), (-4, 4)]
            for left, right in sequence:
                self.send_input_pos(0, left)  # Motor node ID 0
                self.send_input_pos(1, right)  # Motor node ID 1
                time.sleep(3)
                
                # Read feedback for both motors
                for node_id in self.motor_positions:
                    feedback = self.get_encoder_estimates(node_id)
                    if feedback:
                        self.log(f"[ID={node_id}] Pos: {feedback['position']:.3f}, Vel: {feedback['velocity']:.3f}")
                
                time.sleep(2)

            # Step 3: Monitor messages for a bit
            self.log("\n=== Monitoring CAN Messages ===")
            messages = self.read_messages(duration=2.0)
            self.log(f"Received {len(messages)} messages")

            # Step 4: Stop motors
            self.log("\n=== Stopping Motors ===")
            for node_id in self.motor_positions:
                self.send_set_axis_state(node_id, 1)
                time.sleep(0.1)

        except Exception as e:
            self.log(f" Error: {e}")
        finally:
            try:
                self.bus.shutdown()
                self.log("CAN bus shut down cleanly.")
            except Exception:
                pass

class ZLAC8015DController:
    def __init__(self, port: str, device_id: int = 1, baudrate: int = 115200, 
                 wheel_diameter_m: float = 0.20, gear_ratio: float = 1.0, max_rpm: float = 100.0):
        """
        Initialize ZLAC8015D motor controller with optional wheel parameters for physical unit conversion.
        
        Args:
            port: Serial port device path (e.g., '/dev/hubmotor')
            device_id: Modbus device ID (1 or 2)
            baudrate: Serial communication speed (default: 115200)
            wheel_diameter_m: Wheel diameter in meters (default: 0.20m)
            gear_ratio: Motor RPM / Wheel RPM (default: 1.0, no gearing)
            max_rpm: Maximum motor RPM limit (default: 100.0)
        """
        self.port = port
        self.device_id = device_id
        self.baudrate = baudrate
        self.wheel_diameter_m = wheel_diameter_m
        self.gear_ratio = gear_ratio
        self.max_rpm = max_rpm
        self.instr = self._init_instrument(device_id)

    def _init_instrument(self, modbus_id):
        instr = minimalmodbus.Instrument(self.port, modbus_id)
        instr.serial.baudrate = self.baudrate
        instr.serial.bytesize = 8
        instr.serial.parity = minimalmodbus.serial.PARITY_NONE
        instr.serial.stopbits = 1
        instr.serial.timeout = 0.05  # 50ms timeout (was 0.5s) - sufficient for 115200 baud
        instr.mode = minimalmodbus.MODE_RTU
        instr.close_port_after_each_call = False  # Keep port open for faster transactions
        return instr

    # ======== ID Management ========
    def read_device_id(self):
        return self.instr.read_register(0x2001)

    def write_device_id(self, new_id: int):
        self.instr.write_register(0x2001, new_id)
        print(f"Device ID changed to {new_id}")
        self.device_id = new_id
        self.instr = self._init_instrument(new_id)

    def verify_id(self):
        readback = self.read_device_id()
        return readback == self.device_id

    # ======== Control Modes ========
    def set_velocity_mode(self):
        # Batch write: mode=3 (velocity), enable=8
        self.instr.write_registers(0x200D, [3, 8])

    def set_position_mode(self):
        # Batch write: mode=1 (position), enable=8
        self.instr.write_registers(0x200D, [1, 8])

    def set_torque_mode(self):
        # Batch write: mode=4 (torque), enable=8
        self.instr.write_registers(0x200D, [4, 8])

    # ======== Physical Unit Conversion ========
    def configure_wheel(self, wheel_diameter_m: float, gear_ratio: float = 1.0, max_rpm: float = 100.0):
        """
        Configure wheel parameters for m/s to RPM conversion.
        
        Args:
            wheel_diameter_m: Wheel diameter in meters
            gear_ratio: Motor RPM / Wheel RPM (1.0 = direct drive)
            max_rpm: Maximum motor RPM limit
        """
        self.wheel_diameter_m = wheel_diameter_m
        self.gear_ratio = gear_ratio
        self.max_rpm = max_rpm
        print(f"[ZLAC ID={self.device_id}] Wheel config: diameter={wheel_diameter_m:.3f}m, "
              f"gear_ratio={gear_ratio:.2f}, max_rpm={max_rpm:.1f}")

    def mps_to_rpm(self, speed_mps: float) -> int:
        """
        Convert linear velocity (m/s) to motor RPM.
        
        Formula: RPM = (v_mps / wheel_circumference) * 60 * gear_ratio
        
        Args:
            speed_mps: Linear velocity in meters per second
            
        Returns:
            Motor RPM (clamped to ±max_rpm)
        """
        import math
        if self.wheel_diameter_m <= 1e-9:
            print(f"[ZLAC ID={self.device_id}] Warning: wheel_diameter not configured, cannot convert m/s to RPM")
            return 0
        
        wheel_circumference_m = math.pi * self.wheel_diameter_m
        rpm = (speed_mps / wheel_circumference_m) * 60.0 * self.gear_ratio
        
        # Clamp to max RPM
        rpm = max(-self.max_rpm, min(self.max_rpm, rpm))
        return int(rpm)

    # ======== Motor Control Commands ========
    def set_velocity_mps(self, left_mps: float, right_mps: float):
        """
        Set motor velocity using physical units (m/s).
        Requires wheel parameters to be configured via __init__ or configure_wheel().
        
        Args:
            left_mps: Left wheel linear velocity in m/s
            right_mps: Right wheel linear velocity in m/s
        """
        left_rpm = self.mps_to_rpm(left_mps)
        right_rpm = self.mps_to_rpm(right_mps)
        self.set_velocity(left_rpm, right_rpm)
        # print(f"[ZLAC ID={self.device_id}] Velocity (m/s) → Left: {left_mps:.3f} ({left_rpm} RPM), "
        #       f"Right: {right_mps:.3f} ({right_rpm} RPM)")
    
    def set_velocity(self, left_rpm: int, right_rpm: int):
        # Batch write both velocities in one transaction (2x faster)
        self._write_registers_signed(0x2088, [int(left_rpm), int(right_rpm)])
        # print(f" Velocity Mode → Left: {int(left_rpm)}, Right: {int(right_rpm)}")

    def set_position(self, left_pulses: int, right_pulses: int, speed_rpm: int = 100):
        # Batch write all 6 consecutive registers (0x208A-0x208F) in ONE transaction
        # Convert 32-bit signed positions to register pairs
        left_val = left_pulses if left_pulses >= 0 else (1 << 32) + left_pulses
        right_val = right_pulses if right_pulses >= 0 else (1 << 32) + right_pulses
        
        values = [
            (left_val >> 16) & 0xFFFF,   # 0x208A: left high
            left_val & 0xFFFF,            # 0x208B: left low
            (right_val >> 16) & 0xFFFF,  # 0x208C: right high
            right_val & 0xFFFF,           # 0x208D: right low
            speed_rpm,                    # 0x208E: speed left
            speed_rpm                     # 0x208F: speed right
        ]
        self.instr.write_registers(0x208A, values)
        
        # Start commands must be separate (non-consecutive register)
        self.instr.write_register(0x200E, 0x11)  # Start left
        self.instr.write_register(0x200E, 0x12)  # Start right
        #print(f" Position Mode → Left: {left_pulses}, Right: {right_pulses}, Speed: {speed_rpm}")

    def set_torque(self, left_ma: int, right_ma: int):
        # Batch write both torques in one transaction (2x faster)
        self._write_registers_signed(0x2090, [left_ma, right_ma])
        #print(f" Torque Mode → Left: {left_ma}, Right: {right_ma}")

    def stop_motor(self):
        self.instr.write_register(0x200E, 7)
        #print(" Motor stopped")

    # ======== Utility ========
    def _write_registers_signed(self, start_register, values):
        """Write multiple signed 16-bit values in one transaction."""
        unsigned_values = [(v if v >= 0 else (1 << 16) + v) for v in values]
        self.instr.write_registers(start_register, unsigned_values)
    
    def _write_32bit_signed(self, reg_high, reg_low, value):
        """Write a 32-bit signed value across two consecutive registers."""
        if value < 0:
            value = (1 << 32) + value
        high = (value >> 16) & 0xFFFF
        low = value & 0xFFFF
        # Batch write both registers in one transaction
        self.instr.write_registers(reg_high, [high, low])

if __name__ == '__main__':
    # ========== Example 1: CAN Motors (Steadywin with CANABLE 2.0) ==========
    can_motor = MotorCANController(
        channel='/dev/ttyACM0',
        bitrate=500000,
        interface='slcan',
        motor_positions={0: 1.0, 1: -1.0}  # Node IDs: 0 and 1
    )
    can_motor.demo_with_feedback()
    
    # ========== Example 2: ZLAC8015D with Physical Unit Conversion ==========
    # Initialize with wheel parameters for automatic m/s to RPM conversion
    # motor_di = [1, -1]  # Motor direction multipliers
    # zlac_1 = ZLAC8015DController(
    #     port='/dev/hubmotor',
    #     device_id=1,
    #     wheel_diameter_m=0.20,  # 20cm wheels
    #     gear_ratio=1.0,         # Direct drive
    #     max_rpm=100.0           # Max motor speed
    # )
    # zlac_2 = ZLAC8015DController(
    #     port='/dev/hubmotor',
    #     device_id=2,
    #     wheel_diameter_m=0.20,
    #     gear_ratio=1.0,
    #     max_rpm=100.0
    # )
    # 
    # # Set velocity mode
    # zlac_1.set_velocity_mode()
    # zlac_2.set_velocity_mode()
    # 
    # # Control using physical units (m/s)
    # zlac_1.set_velocity_mps(0.5, 0.5)  # 0.5 m/s forward
    # zlac_2.set_velocity_mps(0.5, 0.5)
    # time.sleep(3)
    # 
    # # Or control using RPM directly
    # zlac_1.set_velocity(motor_di[0] * 50, motor_di[1] * 50)
    # zlac_2.set_velocity(motor_di[0] * 50, motor_di[1] * 50)
    # time.sleep(3)
    # 
    # # Stop
    # zlac_1.stop_motor()
    # zlac_2.stop_motor()
    
    # Example: Traditional socketcan (commented out)
    # can_motor = MotorCANController(channel='can0', interface='socketcan')
    # can_motor.demo_sequence()
    
    # Example: ZLAC8015D motors (commented out)
    # motor_di = [1, -1]
    # zlac_1 = ZLAC8015DController('/dev/hubmotor', device_id=1)
    # zlac_2 = ZLAC8015DController('/dev/hubmotor', device_id=2)
    # zlac_1.set_velocity_mode()
    # zlac_2.set_velocity_mode()
    # zlac_1.set_velocity(motor_di[0] * 50, motor_di[1] * 50)
    # zlac_2.set_velocity(motor_di[0] * 50, motor_di[1] * 50)
    # time.sleep(3)
    # zlac_1.stop_motor()
    # zlac_2.stop_motor()


    
    


