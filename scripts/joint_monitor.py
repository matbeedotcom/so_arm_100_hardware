#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys

class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')

        # Joint names for SO-100 arm
        self.joint_names = [
            'Shoulder_Rotation',
            'Shoulder_Pitch',
            'Elbow',
            'Wrist_Pitch',
            'Wrist_Roll',
            'Gripper'
        ]

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Joint State Monitor started. Press Ctrl+C to exit.')
        self.get_logger().info('Move joints manually to record min/max positions.')

    def joint_state_callback(self, msg):
        if len(msg.position) >= 6:
            print('\n' + '='*60)
            print('Joint Positions:')
            print('='*60)

            for i, joint_name in enumerate(self.joint_names):
                if i < len(msg.position):
                    position = msg.position[i]
                    print(f'{joint_name:<18}: {position:>8.4f} rad ({position*57.2958:>7.1f}°)')

            print('='*60)
            print('Record these values for joint_limits.yaml')

def main(args=None):
    rclpy.init(args=args)

    try:
        monitor = JointStateMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print('\nShutdown requested by user.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()