#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import merlin_hw
import time

time.sleep(1)

merlin_bot = merlin_hw.robot()
merlin_bot.start()

def joy_callback(msg):
    # Print axes values
    # for i, axis_value in enumerate(msg.axes):
    #     print(f"Axis {i}: {axis_value}")

    if len(msg.buttons) > 4 and msg.buttons[4] == 1:
        while msg.buttons[4] == 1:
            rospy.loginfo("Rotating Left")
            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=0,rotate_throttle=1)
            pygame.event.pump()
            
    if len(msg.buttons) > 5 and msg.buttons[5] == 1:
        while msg.buttons[5] == 1:
            rospy.loginfo("Rotating Right")
            pygame.event.pump()

    # if len(msg.buttons) > 1 and msg.buttons[1] == 1:
    #     while msg.buttons[1] == 1:
    #         rospy.loginfo("Hello World")
    #         pygame.event.pump()

    # if len(msg.buttons) > 1 and msg.buttons[1] == 1:
    #     while msg.buttons[1] == 1:
    #         rospy.loginfo("Hello World")
    #         pygame.event.pump()

    # if len(msg.buttons) > 1 and msg.buttons[1] == 1:
    #     while msg.buttons[1] == 1:
    #         rospy.loginfo("Hello World")
    #         pygame.event.pump()

    # if len(msg.buttons) > 1 and msg.buttons[1] == 1:
    #     while msg.buttons[1] == 1:
    #         rospy.loginfo("Hello World")
    #         pygame.event.pump()

    # if len(msg.buttons) > 1 and msg.buttons[1] == 1:
    #     while msg.buttons[1] == 1:
    #         rospy.loginfo("Hello World")
    #         pygame.event.pump()

    # if len(msg.buttons) > 1 and msg.buttons[1] == 1:
    #     while msg.buttons[1] == 1:
    #         rospy.loginfo("Hello World")
    #         pygame.event.pump()

    # if len(msg.buttons) > 1 and msg.buttons[1] == 1:
    #     while msg.buttons[1] == 1:
    #         rospy.loginfo("Hello World")
    #         pygame.event.pump()

if __name__ == '__main__':
    try:
        rospy.init_node('ps4_controller_listener', anonymous=True)
        rospy.Subscriber('/ps4_controller', Joy, joy_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass