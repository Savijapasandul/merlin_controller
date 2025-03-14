#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

class ButtonStateTracker:
    def __init__(self):
        # Track states for buttons 1-10
        self.prev_button_states = [False] * 10  # Previous state
        self.curr_button_states = [False] * 10  # Current state

        rospy.init_node('button_state_tracker')
        rospy.Subscriber('/ps4_controller', Joy, self.joy_callback)

    def joy_callback(self, msg):
        # Update button states
        self.prev_button_states = self.curr_button_states.copy()
        
        # Get current states (limit to 10 buttons)
        for i in range(min(len(msg.buttons), 10)):
            self.curr_button_states[i] = msg.buttons[i] == 1

    def is_pressed(self, button_id):
        """Check if button is currently pressed (held down)"""
        return self.curr_button_states[button_id]

    def was_pressed(self, button_id):
        """Check if button was just pressed (transition from released to pressed)"""
        return self.curr_button_states[button_id] and not self.prev_button_states[button_id]

    def was_released(self, button_id):
        """Check if button was just released (transition from pressed to released)"""
        return not self.curr_button_states[button_id] and self.prev_button_states[button_id]

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Example usage (replace with your own logic)
            if self.is_pressed(4):  # Button 5 (index 4)
                print("Button 5: Held down")
                
            if self.was_pressed(5):  # Button 6 (index 5)
                print("Button 6: Just pressed")
                
            if self.was_released(6):  # Button 7 (index 6)
                print("Button 7: Just released")
                
            rate.sleep()

if __name__ == '__main__':
    tracker = ButtonStateTracker()
    tracker.run()