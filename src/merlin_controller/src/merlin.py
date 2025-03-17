#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

# import merlin_hw

# merlin_bot = merlin_hw.robot()
# merlin_bot.start()

class ButtonActionHandler:
    def __init__(self):
        # Track button states (buttons 1-10)
        self.button_states = [False] * 10

        rospy.init_node('button_action_handler')
        rospy.Subscriber('/ps4_controller', Joy, self.joy_callback)

    def joy_callback(self, msg):
        # Update button states (buttons 1-10)
        for i in range(min(len(msg.buttons), 10)):
            self.button_states[i] = msg.buttons[i] == 1

    def run(self):
        
        while not rospy.is_shutdown():
            
            # Rotating left
            if self.button_states[4]:
                while self.button_states[4] and not rospy.is_shutdown():
                    print("Button 5: Continuous action while pressed")
                    if not self.button_states[4]:
                        print("Button 5: released")
                        # merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=0,rotate_throttle=-1)
                        break



            # rate.sleep()

if __name__ == '__main__':
    try:
        handler = ButtonActionHandler()
        handler.run()
    except rospy.ROSInterruptException:
        pass