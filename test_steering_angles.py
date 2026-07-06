#!/usr/bin/env python3
"""
Test script to visualize double Ackermann steering angles
Shows angles for front-left, front-right, rear-left, rear-right
"""

import math

class DoubleAckermannSteering:
    def __init__(self, wheelbase=0.36, track_width=0.36):
        self.L = wheelbase  # wheelbase
        self.W = track_width  # track width
        self.max_steering_angle = 30.0

    def compute_steering_angle(self, R, side):
        """Compute steering angle for a given side (left/right)"""
        offset = -self.W / 2.0 if side == 'left' else self.W / 2.0
        angle = math.degrees(math.atan((self.L / 2.0) / (R + offset)))

        # Clamp to max steering angle
        angle = max(-self.max_steering_angle, min(self.max_steering_angle, angle))

        # Snap to 0 if very small
        if abs(angle) < 0.1:
            angle = 0.0
        return angle

    def compute_wheel_speed(self, R, side):
        """Compute relative wheel speed for a given side"""
        offset = -self.W / 2.0 if side == 'left' else self.W / 2.0
        wheel_radius = math.hypot(R + offset, self.L / 2.0)
        # Relative speed (assuming v_current=1.0, omega=1/R)
        if abs(R) > 1e6:
            return 1.0  # Straight line
        return wheel_radius / abs(R)

    def update(self, R):
        """Calculate all steering angles and speeds for turn radius R"""
        # Steering angles using the FIXED formula (opposite offsets for rear)
        angle_fl = self.compute_steering_angle(R, 'left')
        angle_fr = self.compute_steering_angle(R, 'right')
        angle_rl = self.compute_steering_angle(R, 'right')  # opposite offset
        angle_rr = self.compute_steering_angle(R, 'left')   # opposite offset

        # Wheel speeds
        speed_fl = self.compute_wheel_speed(R, 'left')
        speed_fr = self.compute_wheel_speed(R, 'right')
        speed_rl = self.compute_wheel_speed(R, 'right')  # opposite offset
        speed_rr = self.compute_wheel_speed(R, 'left')   # opposite offset

        return {
            'angle_fl': angle_fl, 'angle_fr': angle_fr, 'angle_rl': angle_rl, 'angle_rr': angle_rr,
            'speed_fl': speed_fl, 'speed_fr': speed_fr, 'speed_rl': speed_rl, 'speed_rr': speed_rr
        }

def print_turn_scenario(robot, name, R):
    """Print steering angles and speeds for a turn scenario"""
    result = robot.update(R)
    print(f"\n{'='*70}")
    print(f"{name} (Turn radius R = {R:.2f}m)")
    print(f"{'='*70}")
    print(f"STEERING ANGLES (degrees):")
    print(f"  Front-Left:  {result['angle_fl']:7.2f}°  |  Front-Right: {result['angle_fr']:7.2f}°")
    print(f"  Rear-Left:   {result['angle_rl']:7.2f}°  |  Rear-Right:  {result['angle_rr']:7.2f}°")
    print(f"\nWHEEL SPEEDS (relative, v=1.0):")
    print(f"  Front-Left:  {result['speed_fl']:7.3f}  |  Front-Right: {result['speed_fr']:7.3f}")
    print(f"  Rear-Left:   {result['speed_rl']:7.3f}  |  Rear-Right:  {result['speed_rr']:7.3f}")
    print(f"\nSPEED DIFFERENCES:")
    print(f"  Left side:  {result['speed_fl']:.3f} vs {result['speed_rl']:.3f} (same)")
    print(f"  Right side: {result['speed_fr']:.3f} vs {result['speed_rr']:.3f} (same)")
    print(f"  Outside/Inside ratio: {max(result['speed_fr'], result['speed_rr']) / max(result['speed_fl'], result['speed_rl']):.3f}")

if __name__ == "__main__":
    robot = DoubleAckermannSteering(wheelbase=0.36, track_width=0.36)

    print("\n")
    print("╔════════════════════════════════════════════════════════════════════╗")
    print("║  DOUBLE ACKERMANN STEERING TEST (Fixed Version)                   ║")
    print("║  Wheelbase: 0.36m  |  Track Width: 0.36m  |  Max Angle: 30°      ║")
    print("╚════════════════════════════════════════════════════════════════════╝")

    # Scenario 1: Straight ahead (infinite radius)
    print_turn_scenario(robot, "STRAIGHT (Center)", 1e6)

    # Scenario 2: Gentle right turn (large radius)
    print_turn_scenario(robot, "GENTLE RIGHT TURN", 5.0)

    # Scenario 3: Moderate right turn (medium radius)
    print_turn_scenario(robot, "MODERATE RIGHT TURN", 2.0)

    # Scenario 4: Tight right turn (small radius)
    print_turn_scenario(robot, "TIGHT RIGHT TURN", 0.8)

    # Scenario 5: Gentle left turn (negative radius)
    print_turn_scenario(robot, "GENTLE LEFT TURN", -5.0)

    # Scenario 6: Moderate left turn
    print_turn_scenario(robot, "MODERATE LEFT TURN", -2.0)

    # Scenario 7: Tight left turn
    print_turn_scenario(robot, "TIGHT LEFT TURN", -0.8)

    print(f"\n{'='*70}")
    print("KEY OBSERVATIONS:")
    print(f"{'='*70}")
    print("✓ All steering angles should be DIFFERENT between front-left/right")
    print("✓ Rear angles should be OPPOSITE direction to front angles")
    print("✓ Left wheel speeds should be different from right wheel speeds")
    print("✓ Outside wheels travel faster than inside wheels")
    print(f"{'='*70}\n")
