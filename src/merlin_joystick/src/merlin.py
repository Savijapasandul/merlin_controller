#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import merlin_hw

merlin_bot = merlin_hw.robot()
merlin_bot.start()

def joy_callback(msg):

    buttons = msg.buttons
    axes = msg.axes
    fwd_bwd_throttle = axes[1]
    left_right_throttle = axes[0]
    
    if fwd_bwd_throttle == -1.0 and left_right_throttle == -1.0:
        #merlin_bot.set_velocity_throttle(fwd_bwd_throttle=1,left_right_throttle=1,rotate_throttle=0)
        rospy.loginfo("Moving Forward-Left")
    elif fwd_bwd_throttle == -1.0 and left_right_throttle == 0.999969482421875:
        #merlin_bot.set_velocity_throttle(fwd_bwd_throttle=1,left_right_throttle=-1,rotate_throttle=0)
        rospy.loginfo("Moving Forward-Right")
    elif fwd_bwd_throttle == 0.999969482421875 and left_right_throttle == -1.0:
        #merlin_bot.set_velocity_throttle(fwd_bwd_throttle=-1,left_right_throttle=1,rotate_throttle=0)
        rospy.loginfo("Moving Backward-Left")
    elif fwd_bwd_throttle == 0.999969482421875 and left_right_throttle == 0.999969482421875:
        #merlin_bot.set_velocity_throttle(fwd_bwd_throttle=-1,left_right_throttle=-1,rotate_throttle=0)
        rospy.loginfo("Moving Backward-Right")
    elif fwd_bwd_throttle == -1.0:
        #merlin_bot.set_velocity_throttle(fwd_bwd_throttle=1,left_right_throttle=0,rotate_throttle=0)
        rospy.loginfo("Moving Forward")
    elif fwd_bwd_throttle == 0.999969482421875:
        #merlin_bot.set_velocity_throttle(fwd_bwd_throttle=-1,left_right_throttle=0,rotate_throttle=0)
        rospy.loginfo("Moving Backward")
    elif left_right_throttle == -1.0:
        #merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=1,rotate_throttle=0)
        rospy.loginfo("Moving Left")
    elif left_right_throttle == 0.999969482421875:
        #merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=-1,rotate_throttle=0)
        rospy.loginfo("Moving Right")
    elif buttons[4] == 1:
        rospy.loginfo("Rotating Left")
    elif buttons[5] == 1:
        rospy.loginfo("Rotating Right")
    elif buttons[0] == 1:
        rospy.loginfo("0R")
    elif buttons[2] == 1:
        rospy.loginfo("0R")
    elif buttons[1] == 1:
        rospy.loginfo("1R")
    elif buttons[3] == 1:
        rospy.loginfo("1R")
    elif buttons[6] == 1:
        rospy.loginfo("2R")
    elif buttons[7] == 1:
        rospy.loginfo("2R")
    elif buttons[9] == 1:
        rospy.loginfo("gripper open")
    elif buttons[10] == 1:
        rospy.loginfo("gripper close")
    else:
        #merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=0,rotate_throttle=0)
        pass

    
    # Command the robot to move
      #merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=0,rotate_throttle=0) #range -1 to 1
      #merlin_bot.set_joint_pos(joint0_val=0, joint1_val=0,joint2_val=0,joint3_val=0)  #range -89 to 89

if __name__ == '__main__':
    rospy.init_node('ps4_controller_listener', anonymous=True)    
    rospy.Subscriber('/ps4_controller', Joy, joy_callback)
    rospy.loginfo("Received joystick input, pls move joystick to start")

    rospy.spin()
