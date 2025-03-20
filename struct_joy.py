import struct
import merlin_hw

# # Initialize the robot
# merlin_bot = merlin_hw.robot()
# merlin_bot.start()

# # Set the initial position of the robot's arm
# merlin_bot.set_joint_pos(joint0_val=0, joint1_val=0, joint2_val=89, joint3_val=0)

# Path to the joystick device
device_path = "/dev/input/js0"

# Open the joystick device
joystick = open(device_path, "rb")

# Define the event format
EVENT_FORMAT = "IhBB"  # time, value, type, number
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

# Mapping for buttons
button_map = {
    0: "Button Y",
    1: "Button B",
    2: "Button A",
    3: "Button X",
    4: "L1",
    5: "R1",
    6: "L2",
    7: "R2",
    8: "Select",
    9: "Start",
    10: "Left Stick button",
    11: "Right Stick button",
    12: "Home",
}

# Mapping for axes and directions
axis_map = {
    0: "X-Axis (Left/Right)",
    1: "Y-Axis (Forward/Backward)",
}

