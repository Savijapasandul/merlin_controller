#!/usr/bin/env python3

import rospy
import pygame
from std_msgs.msg import String
import json

def main():
    rospy.init_node("joystick_publisher", anonymous=True)
    axis_pub = rospy.Publisher("/joystick_axis", String, queue_size=10)
    button_pub = rospy.Publisher("/joystick_buttons", String, queue_size=10)
    
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        rospy.logerr("No joystick detected.")
        return
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    rospy.loginfo(f"Joystick Name: {joystick.get_name()}")
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        pygame.event.pump()
        
        axis_data = {f"axis_{i}": joystick.get_axis(i) for i in range(joystick.get_numaxes())}
        button_data = {f"button_{i}": joystick.get_button(i) for i in range(joystick.get_numbuttons())}
        
        print("Axis Data:", axis_data)
        print("Button Data:", button_data)
        
        axis_pub.publish(json.dumps(axis_data))
        button_pub.publish(json.dumps(button_data))
        
        rate.sleep()
    
    pygame.quit()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
