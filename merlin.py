import merlin_hw
import time
from importlib import reload
reload(merlin_hw)
time.sleep(1)
merlin_bot=merlin_hw.robot()
control_merlin=merlin_bot.get_robot_control_layout()
merlin_bot.start()
merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0,left_right_throttle=0,rotate_throttle=0) #range -1 to 1
merlin_bot.set_joint_pos(joint0_val=0, joint1_val=0,joint2_val=89,joint3_val=0)  #range -89 to 89