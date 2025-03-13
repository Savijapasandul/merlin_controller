#!/usr/bin/env python3

import rospy
import pygame
from std_msgs.msg import String
from sensor_msgs.msg import Joy

pygame.init()
pygame.joystick.init()

rospy.init_node('ps4_controller_node', anonymous=True)
joy_pub = rospy.Publisher('/ps4_controller', Joy, queue_size=10)

if pygame.joystick.get_count() == 0:
    rospy.loginfo("No joystick detected.")
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    rospy.loginfo(f"Joystick Name: {joystick.get_name()}")

    joy_msg = Joy()

    # Loop to continuously check for events and publish joystick data
    rate = rospy.Rate(10)  # Publish at 10 Hz
    while not rospy.is_shutdown():
        pygame.event.pump()
        
        buttons = []
        for i in range(joystick.get_numbuttons()):
            buttons.append(joystick.get_button(i))

        axes = []
        for i in range(joystick.get_numaxes()):
            axes.append(joystick.get_axis(i))
        
        joy_msg.axes = axes
        joy_msg.buttons = buttons

        joy_pub.publish(joy_msg)

        # Sleep to maintain the desired loop rate
        rate.sleep()

    rospy.spin()

