#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import merlin_hw

merlin_bot = merlin_hw.robot()
merlin_bot.start()

class ButtonActionHandler:
    def __init__(self):
        self.button_states = [False] * 10
        rospy.init_node('button_action_handler')
        rospy.Subscriber('/ps4_controller', Joy, self.joy_callback)

    def joy_callback(self, msg):
        for i in range(min(len(msg.buttons), 10)):
            self.button_states[i] = msg.buttons[i] == 1

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            # Rotating left
            if self.button_states[4]:
                while self.button_states[4] and not rospy.is_shutdown():
                    print("Button index 4 pressed: Rotating left")
                    merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=0,rotate_throttle=-1)
                    if not self.button_states[4]:
                        print("Button 4 released: Stopped rotating left")
                        merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=0,rotate_throttle=0)
                        break
                    
            # Rotating right
            if self.button_states[5]:
                while self.button_states[5] and not rospy.is_shutdown():
                    print("Button index 5 pressed: Rotating right")
                    merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=0,rotate_throttle=1)
                    if not self.button_states[5]:
                        print("Button 5 released: Stopped rotating right")
                        merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=0,rotate_throttle=0)
                        break
            rate.sleep()

if __name__ == '__main__':
    try:
        handler = ButtonActionHandler()
        handler.run()
    except rospy.ROSInterruptException:
        pass