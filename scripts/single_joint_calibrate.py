#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
import sys
import yaml
import os
from datetime import datetime

class SingleJointCalibrator(Node):
    def __init__(self):
        super().__init__('single_joint_calibrator')

        # Available joints
        self.joint_names = [
            'Shoulder_Rotation',    # 0
            'Shoulder_Pitch',       # 1
            'Elbow',               # 2
            'Wrist_Pitch',         # 3
            'Wrist_Roll',          # 4
            'Gripper'              # 5
        ]

        # Create service clients
        self.torque_client = self.create_client(Trigger, 'toggle_torque')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_positions = [0.0] * 6
        self.get_logger().info('Single Joint Calibrator ready')

    def joint_state_callback(self, msg):
        if len(msg.position) >= 6:
            self.current_positions = list(msg.position[:6])

    def disable_torque(self):
        """Disable servo torque to allow manual movement"""
        print("\nDisabling torque...")

        if not self.torque_client.wait_for_service(timeout_sec=2.0):
            print("Toggle torque service not available!")
            return False

        try:
            future = self.torque_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.result():
                response = future.result()
                if response.success:
                    print("✓ Torque disabled - you can now move joints manually")
                    return True
                else:
                    print(f"Failed to disable torque: {response.message}")
            else:
                print("Service call failed or timed out")
        except Exception as e:
            print(f"Error calling service: {e}")

        return False

    def enable_torque(self):
        """Re-enable servo torque"""
        print("\nRe-enabling torque...")

        try:
            future = self.torque_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.result():
                response = future.result()
                if response.success:
                    print("✓ Torque enabled")
                    return True
                else:
                    print(f"Failed to enable torque: {response.message}")
        except Exception as e:
            print(f"Error calling service: {e}")

        return False

    def get_current_position(self, joint_idx):
        """Get current position of specified joint"""
        # Spin once to get latest joint state
        rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_positions[joint_idx]

    def calibrate_joint(self, joint_idx):
        """Calibrate a single joint"""
        joint_name = self.joint_names[joint_idx]

        print(f"\n{'='*50}")
        print(f"Calibrating {joint_name} (Joint {joint_idx})")
        print(f"{'='*50}")

        # Disable torque
        if not self.disable_torque():
            return None

        limits = {}

        # Get minimum position
        print(f"\n1. Move {joint_name} to MINIMUM position (mechanical limit)")
        input("Press Enter when ready...")
        limits['min'] = self.get_current_position(joint_idx)
        print(f"Min position recorded: {limits['min']:.4f} rad ({limits['min']*57.2958:.1f}°)")

        # Get center position
        print(f"\n2. Move {joint_name} to CENTER/NEUTRAL position")
        input("Press Enter when ready...")
        limits['center'] = self.get_current_position(joint_idx)
        print(f"Center position recorded: {limits['center']:.4f} rad ({limits['center']*57.2958:.1f}°)")

        # Get maximum position
        print(f"\n3. Move {joint_name} to MAXIMUM position (mechanical limit)")
        input("Press Enter when ready...")
        limits['max'] = self.get_current_position(joint_idx)
        print(f"Max position recorded: {limits['max']:.4f} rad ({limits['max']*57.2958:.1f}°)")

        # Calculate range
        range_rad = limits['max'] - limits['min']
        range_deg = range_rad * 57.2958

        print(f"\n{joint_name} Calibration Results:")
        print(f"  Min:    {limits['min']:>8.4f} rad ({limits['min']*57.2958:>7.1f}°)")
        print(f"  Center: {limits['center']:>8.4f} rad ({limits['center']*57.2958:>7.1f}°)")
        print(f"  Max:    {limits['max']:>8.4f} rad ({limits['max']*57.2958:>7.1f}°)")
        print(f"  Range:  {range_rad:>8.4f} rad ({range_deg:>7.1f}°)")

        # Re-enable torque
        self.enable_torque()

        return limits

    def save_calibration(self, joint_idx, limits):
        """Save calibration data to files"""
        joint_name = self.joint_names[joint_idx]

        # Create calibration data structure
        calibration_data = {
            'timestamp': datetime.now().isoformat(),
            'joint': joint_name,
            'limits': {
                'min_position': limits['min'],
                'center_position': limits['center'],
                'max_position': limits['max'],
                'range_rad': limits['max'] - limits['min'],
                'range_deg': (limits['max'] - limits['min']) * 57.2958
            }
        }

        # Save to calibration file
        config_dir = '/ros2_ws/src/so_arm_100_hardware/config'
        os.makedirs(config_dir, exist_ok=True)

        calib_file = os.path.join(config_dir, f'{joint_name.lower()}_calibration.yaml')
        with open(calib_file, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False)

        print(f"\n✓ Calibration data saved to: {calib_file}")

        # Update or create joint_limits.yaml entry
        joint_limits_file = '/ros2_ws/src/so_100_arm/config/joint_limits.yaml'

        try:
            # Load existing joint_limits.yaml
            if os.path.exists(joint_limits_file):
                with open(joint_limits_file, 'r') as f:
                    joint_limits = yaml.safe_load(f) or {}
            else:
                joint_limits = {'joint_limits': {}}

            # Ensure joint_limits structure exists
            if 'joint_limits' not in joint_limits:
                joint_limits['joint_limits'] = {}

            # Update this joint's limits
            joint_limits['joint_limits'][joint_name] = {
                'max_position': limits['max'],
                'min_position': limits['min'],
                'max_velocity': 1.0,  # Default velocity limit
                'max_effort': 10.0    # Default effort limit
            }

            # Save updated joint_limits.yaml
            with open(joint_limits_file, 'w') as f:
                yaml.dump(joint_limits, f, default_flow_style=False)

            print(f"✓ Updated joint_limits.yaml with {joint_name} limits")

        except Exception as e:
            print(f"Warning: Could not update joint_limits.yaml: {e}")
            print("Manual update required:")
            print(f"  {joint_name}:")
            print(f"    max_position: {limits['max']:.6f}")
            print(f"    min_position: {limits['min']:.6f}")

def main():
    rclpy.init()

    calibrator = SingleJointCalibrator()

    # Display joint options
    print("\nAvailable joints:")
    for i, joint in enumerate(calibrator.joint_names):
        print(f"  {i}: {joint}")

    try:
        joint_idx = int(input("\nEnter joint number to calibrate (0-5): "))

        if 0 <= joint_idx <= 5:
            limits = calibrator.calibrate_joint(joint_idx)

            if limits:
                # Save calibration data
                calibrator.save_calibration(joint_idx, limits)

                print("\n" + "="*50)
                print("CALIBRATION COMPLETE")
                print("="*50)
                print(f"Calibration data automatically saved for {calibrator.joint_names[joint_idx]}")
                print(f"Values written to joint_limits.yaml:")
                print(f"  max_position: {limits['max']:.6f}")
                print(f"  min_position: {limits['min']:.6f}")

        else:
            print("Invalid joint number!")

    except ValueError:
        print("Please enter a valid number!")
    except KeyboardInterrupt:
        print("\nCalibration cancelled by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()