#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  # Standard ROS message for joystick data
import pygame

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')

        # Create a publisher for joystick messages
        self.joy_pub = self.create_publisher(Joy, 'joystick_data', 10)

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected.")
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Connected to joystick: {self.joystick.get_name()}")

        self.timer = self.create_timer(0.1, self.publish_joystick_data)  # 10Hz update

    def publish_joystick_data(self):
        pygame.event.pump()  # Process pygame events

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Read axis values
        msg.axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]

        # Read button states
        msg.buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

        self.joy_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()
