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
        # rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            
            # Rotating left
            if self.button_states[4]:
                # while self.button_states[4]:
                #     print("Button index 4 pressed: Rotating left")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=-1)
                # if not self.button_states[4]:
                #     print("Button 4 released: Stopped rotating left")
                #     merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)
                
            # Rotating right
            if self.button_states[5]:
                # while self.button_states[5]:
                #     print("Button index 5 pressed: Rotating right")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=1)
                # if not self.button_states[5]:
                #     print("Button 5 released: Stopped rotating right")
                #     merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)

            # fwd
            if self.button_states[6]:
                # while self.button_states[6]:
                #     print("Button index 6 pressed: Rotating right")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=-1, left_right_throttle=0, rotate_throttle=0)
                # if not self.button_states[6]:
                #     print("Button 6 released: Stopped rotating right")
                #     merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)

            # bwd
            if self.button_states[7]:
                # while self.button_states[7]:
                #     print("Button index 7 pressed: Rotating right")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=1, left_right_throttle=0, rotate_throttle=0)
                # if not self.button_states[7]:
                #     print("Button 7 released: Stopped rotating right")
                #     merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)

            # left
            if self.button_states[3]:
                # while self.button_states[3]:
                #     print("Button index 3 pressed: Rotating right")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=-1, rotate_throttle=0)
                # if not self.button_states[3]:
                #     print("Button 3 released: Stopped rotating right")
                #     merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)

            # right
            if self.button_states[1]:
                # while self.button_states[1]:
                #     print("Button index 1 pressed: Rotating right")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=1, rotate_throttle=0)
                # if not self.button_states[1]:
                #     print("Button 1 released: Stopped rotating right")
                #     merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)

            # stop
            if self.button_states[0]:
                # while self.button_states[0]:
                #     print("Button index 0 pressed: Rotating right")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)
                # if not self.button_states[0]:
                #     print("Button 0 released: Stopped rotating right")
                #     merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)


            # rate.sleep()

if __name__ == '__main__':
    try:
        handler = ButtonActionHandler()
        handler.run()
    except KeyboardInterrupt:
        print("Ctrl+C pressed. Shutting down...")
        handler.robot_shutdown()
        rospy.signal_shutdown("Ctrl+C pressed")
