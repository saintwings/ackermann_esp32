#!/usr/bin/env python3

import os
import time
import struct
import subprocess

import can

# ============================================================
# CONFIG
# ============================================================

CAN_INTERFACE = "can0"
CAN_BITRATE = 1000000

MOTOR_ID = 0x01

# ============================================================
# CAN BUS ACTIVATION
# ============================================================

def setup_can_interface():
    """
    Bring up SocketCAN interface.
    """

    print("\n===================================")
    print("SETUP CAN INTERFACE")
    print("===================================")

    # bring interface down first
    subprocess.run(
        ["sudo", "ip", "link", "set", CAN_INTERFACE, "down"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    # set bitrate + up
    cmd = [
        "sudo",
        "ip",
        "link",
        "set",
        CAN_INTERFACE,
        "up",
        "type",
        "can",
        "bitrate",
        str(CAN_BITRATE)
    ]

    result = subprocess.run(cmd)

    if result.returncode != 0:
        raise RuntimeError("Failed to setup CAN interface")

    print(f"CAN Interface : {CAN_INTERFACE}")
    print(f"Bitrate       : {CAN_BITRATE}")

    time.sleep(1)


def show_can_status():

    print("\n===================================")
    print("CAN STATUS")
    print("===================================")

    subprocess.run(
        ["ip", "-details", "link", "show", CAN_INTERFACE]
    )


# ============================================================
# CREATE CAN BUS
# ============================================================

def create_bus():

    print("\nConnecting to CAN bus...")

    bus = can.interface.Bus(
        channel=CAN_INTERFACE,
        interface="socketcan"
    )

    print("CAN bus connected")

    return bus


# ============================================================
# LOW LEVEL CAN
# ============================================================

class MotorController:

    def __init__(self, bus, motor_id=0x01):

        self.bus = bus
        self.motor_id = motor_id

    # --------------------------------------------------------

    def send_cmd(self, cmd, payload=b'', can_id=None):

        if can_id is None:
            can_id = self.motor_id

        data = bytes([cmd]) + payload

        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False
        )

        self.bus.send(msg)

        print(f"\nTX -> ID={hex(can_id)} DATA={[hex(x) for x in data]}")

    # --------------------------------------------------------

    def recv(self, timeout=1.0):

        msg = self.bus.recv(timeout)

        if msg is None:
            print("No response")
            return None

        print(f"RX <- ID={hex(msg.arbitration_id)} DATA={[hex(x) for x in msg.data]}")

        return msg

    # ========================================================
    # UTILITY
    # ========================================================

    def degree_to_count(self, deg):

        return int(deg * 16384 / 360.0)

    def count_to_degree(self, count):

        return count * 360.0 / 16384.0

    # ========================================================
    # SYSTEM
    # ========================================================

    def reboot_motor(self):

        self.send_cmd(
            0x00,
            bytes([0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF])
        )

        print("Motor reboot command sent")

    def clear_fault(self):

        self.send_cmd(0xAF)

        self.recv()

    def disable_motor(self):

        self.send_cmd(0xCF)

        self.recv()

    # ========================================================
    # VERSION / INFO
    # ========================================================

    def read_versions(self):

        self.send_cmd(0xA0)

        msg = self.recv()

        if not msg:
            return

        data = msg.data

        boot_ver = struct.unpack("<H", data[1:3])[0]
        app_ver = struct.unpack("<H", data[3:5])[0]
        hw_ver = struct.unpack("<H", data[5:7])[0]
        can_proto = data[7]

        print("\n========== VERSION INFO ==========")
        print("Boot Version :", boot_ver)
        print("App Version  :", app_ver)
        print("HW Version   :", hw_ver)
        print("CAN Protocol :", can_proto)

        return {
            "boot_ver": boot_ver,
            "app_ver": app_ver,
            "hw_ver": hw_ver,
            "can_proto": can_proto,
        }

    # --------------------------------------------------------

    def read_motor_info(self):

        self.send_cmd(0xB0)

        msg = self.recv()

        if not msg:
            return

        data = msg.data

        pole_pairs = data[1]

        torque_constant = struct.unpack("<f", data[2:6])[0]

        gear_ratio = data[6]

        print("\n========== MOTOR INFO ==========")
        print("Pole Pairs      :", pole_pairs)
        print("Torque Constant :", torque_constant, "Nm/A")
        print("Gear Ratio      :", gear_ratio)

        return {
            "pole_pairs": pole_pairs,
            "torque_constant": torque_constant,
            "gear_ratio": gear_ratio,
        }

    # ========================================================
    # STATUS
    # ========================================================

    def decode_fault(self, fault):

        print("\nFault Decode:")

        if fault == 0:
            print("No Fault")
            return

        if fault & (1 << 0):
            print("- Voltage Fault")

        if fault & (1 << 1):
            print("- Current Fault")

        if fault & (1 << 2):
            print("- Temperature Fault")

        if fault & (1 << 3):
            print("- Encoder Fault")

        if fault & (1 << 6):
            print("- Hardware Fault")

        if fault & (1 << 7):
            print("- Software Fault")

    # --------------------------------------------------------

    def read_status(self):

        self.send_cmd(0xAE)

        msg = self.recv()

        if not msg:
            return

        data = msg.data

        voltage = struct.unpack("<H", data[1:3])[0] * 0.01

        current = struct.unpack("<H", data[3:5])[0] * 0.01

        temp = data[5]

        mode = data[6]

        fault = data[7]

        mode_names = {
            0: "Disabled",
            1: "Voltage",
            2: "Current",
            3: "Speed",
            4: "Position"
        }

        print("\n========== STATUS ==========")
        print("Voltage :", voltage, "V")
        print("Current :", current, "A")
        print("Temp    :", temp, "C")
        print("Mode    :", mode_names.get(mode, "Unknown"))
        print("Fault   :", bin(fault))

        self.decode_fault(fault)

        return {
            "voltage": voltage,
            "current": current,
            "temp": temp,
            "mode": mode_names.get(mode, "Unknown"),
            "fault": fault,
        }

    # ========================================================
    # REALTIME DATA
    # ========================================================

    def read_current(self):

        self.send_cmd(0xA1)

        msg = self.recv()

        if not msg:
            return

        current_raw = struct.unpack("<i", msg.data[1:5])[0]

        current = current_raw * 0.001

        print("\n========== CURRENT ==========")
        print("Q Current :", current, "A")

    # --------------------------------------------------------

    def read_speed(self):

        self.send_cmd(0xA2)

        msg = self.recv()

        if not msg:
            return

        speed_raw = struct.unpack("<i", msg.data[1:5])[0]

        rpm = speed_raw * 0.01

        print("\n========== SPEED ==========")
        print("RPM :", rpm)

    # --------------------------------------------------------

    def read_position(self):

        self.send_cmd(0xA3)

        msg = self.recv()

        if not msg:
            return

        data = msg.data

        single_turn = struct.unpack("<H", data[1:3])[0]

        multi_turn = struct.unpack("<i", data[3:7])[0]

        single_deg = self.count_to_degree(single_turn)

        total_deg = self.count_to_degree(multi_turn)

        print("\n========== POSITION ==========")
        print("Single Turn :", single_deg, "deg")
        print("Total Angle :", total_deg, "deg")

        return {
            "single_deg": single_deg,
            "total_deg": total_deg,
        }

    # --------------------------------------------------------

    def read_realtime(self):

        self.send_cmd(0xA4)

        msg = self.recv()

        if not msg:
            return

        data = msg.data

        temp = data[1]

        current = struct.unpack("<h", data[2:4])[0] * 0.001

        speed = struct.unpack("<h", data[4:6])[0] * 0.01

        angle_raw = struct.unpack("<H", data[6:8])[0]

        angle_deg = self.count_to_degree(angle_raw)

        print("\n========== REALTIME ==========")
        print("Temp    :", temp, "C")
        print("Current :", current, "A")
        print("Speed   :", speed, "RPM")
        print("Angle   :", angle_deg, "deg")

        return {
            "temp": temp,
            "current": current,
            "speed": speed,
            "angle_deg": angle_deg,
        }

    # ========================================================
    # CONFIGURATION
    # ========================================================

    def set_current_position_as_zero(self):

        print("\n===================================")
        print("SET ZERO POSITION")
        print("===================================")

        self.send_cmd(0xB1)

        msg = self.recv(timeout=2.0)

        if not msg:
            print("Failed to set zero")
            return

        data = msg.data

        if len(data) >= 3:

            offset = struct.unpack("<H", data[1:3])[0]

            print(f"Encoder offset stored : {offset}")

        print("Current position is now ZERO")

    def set_position_max_speed(self, rpm):

        value = int(rpm * 100)

        payload = struct.pack("<I", value)

        self.send_cmd(0xB2, payload)

        self.recv()

    # --------------------------------------------------------

    def set_max_current(self, current_amp):

        value = int(current_amp * 1000)

        payload = struct.pack("<I", value)

        self.send_cmd(0xB3, payload)

        self.recv()

    # --------------------------------------------------------

    def set_acceleration(self, rpm_per_sec):

        value = int(rpm_per_sec * 100)

        payload = struct.pack("<I", value)

        self.send_cmd(0xB5, payload)

        self.recv()

    # ========================================================
    # CONTROL
    # ========================================================

    def torque_control(self, current_amp):

        value = int(current_amp * 1000)

        payload = struct.pack("<i", value)

        self.send_cmd(0xC0, payload)

        self.recv()

    # --------------------------------------------------------

    def speed_control(self, rpm):

        value = int(rpm * 100)

        payload = struct.pack("<i", value)

        self.send_cmd(0xC1, payload)

        self.recv()

    # --------------------------------------------------------

    def absolute_position_control(self, degree):

        counts = self.degree_to_count(degree)

        payload = struct.pack("<i", counts)

        self.send_cmd(0xC2, payload)

        self.recv()

    # --------------------------------------------------------

    def relative_position_control(self, degree):

        counts = self.degree_to_count(degree)

        payload = struct.pack("<i", counts)

        self.send_cmd(0xC3, payload)

        self.recv()

    # --------------------------------------------------------

    def go_home(self, settle_time=2.0):

        # Home command is non-blocking on the drive; wait briefly so
        # follow-up commands do not immediately override the homing move.
        self.send_cmd(0xC4)

        self.recv(timeout=0.5)

        if settle_time > 0:
            time.sleep(settle_time)

    # ========================================================
    # MONITOR
    # ========================================================

    def monitor(self, interval=0.5):

        try:

            while True:

                self.read_realtime()

                time.sleep(interval)

        except KeyboardInterrupt:

            print("\nMonitor stopped")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    print("===================================")
    print("MOTOR CAN CONTROLLER")
    print("===================================")

    # --------------------------------------------------------
    # ACTIVATE CAN BUS
    # --------------------------------------------------------

    setup_can_interface()

    show_can_status()

    # --------------------------------------------------------
    # CREATE BUS
    # --------------------------------------------------------

    bus = create_bus()

    motor_1 = MotorController(
        bus=bus,
        motor_id=0x01
    )

    motor_2 = MotorController(
        bus=bus,
        motor_id=0x02
    )

    # --------------------------------------------------------
    # CLEAR FAULT
    # --------------------------------------------------------

    motor_1.clear_fault()
    motor_2.clear_fault()
    time.sleep(1)

    # # --------------------------------------------------------
    # # READ INFO
    # # --------------------------------------------------------

    # motor.read_versions()

    # motor.read_motor_info()

    # motor.read_status()

    # motor.read_realtime()

    # # --------------------------------------------------------
    # # CONFIG
    # # --------------------------------------------------------

    # motor.set_position_max_speed(100)

    # motor.set_max_current(5.0)

    # motor.set_acceleration(200)

    # motor.set_current_position_as_zero()
    # time.sleep(3)
    
    motor_1.read_position()
    motor_2.read_position()




    print("\n Go home")
    motor_1.go_home()
    motor_2.go_home()
    time.sleep(3)

    print("\nMove to 45 deg")
    motor_1.absolute_position_control(45)
    motor_2.absolute_position_control(45)
    motor_1.read_position()
    motor_2.read_position()
    time.sleep(3)

    # print("\nMove to 0 deg")
    # motor.absolute_position_control(0)
    # motor.read_position()
    # time.sleep(3)


    print("\n Go home")
    motor_1.go_home()
    motor_2.go_home()
    time.sleep(3)



    print("\nDisable motor")

    motor_1.disable_motor()
    motor_2.disable_motor()

    print("\nDone")

    # # --------------------------------------------------------
    # # SPEED TEST
    # # --------------------------------------------------------

    # print("\n===================================")
    # print("SPEED CONTROL TEST")
    # print("===================================")

    # motor.speed_control(50)

    # time.sleep(3)

    # motor.read_speed()

    # --------------------------------------------------------
    # POSITION TEST
    # --------------------------------------------------------

    # motor.set_current_position_as_zero()
    # time.sleep(3)
    # for i in range(0,10):
    #     motor.read_position()
    #     time.sleep(3)

    # print("\n===================================")
    # print("POSITION CONTROL TEST")
    # print("===================================")


    # print("\n Go home")
    # motor.go_home()

    # # print("\nMove to 0 deg")
    # # motor.absolute_position_control(0)
    # time.sleep(3)
    # motor.read_position()

    # print("\nMove to 90 deg")
    # motor.absolute_position_control(90)
    # time.sleep(3)
    # motor.read_position()

    # print("\nMove to 180 deg")
    # motor.absolute_position_control(180)
    # time.sleep(3)
    # motor.read_position()

    # print("\nMove to 270 deg")
    # motor.absolute_position_control(270)
    # time.sleep(3)
    # motor.read_position()

    # print("\nMove to 360 deg")
    # motor.absolute_position_control(360)
    # time.sleep(3)
    # motor.read_position()

    # #######################

    # # print("\nMove +90 deg")

    # # motor.relative_position_control(90)

    # # time.sleep(3)

    # # motor.read_position()

    # # print("\nGo home")

    # # motor.go_home()

    # # time.sleep(3)

    # # motor.read_position()

    # # # --------------------------------------------------------
    # # # TORQUE TEST
    # # # --------------------------------------------------------

    # # print("\n===================================")
    # # print("TORQUE TEST")
    # # print("===================================")

    # # motor.torque_control(1.0)

    # # time.sleep(2)

    # # motor.read_current()

    # # --------------------------------------------------------
    # # DISABLE
    # # --------------------------------------------------------

    