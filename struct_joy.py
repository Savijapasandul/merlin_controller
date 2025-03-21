# Description: This script reads the joystick input from a PS4 controller and controls the Merlin robot.
# Author: Savija Pasandul
# Last updated: 2021-08-10

# Instructions for the script:

# 1. Ensure the `merlin_hw` module is installed and accessible in your Python environment.
# 2. Connect the joystick to your system and verify its device path (default is `/dev/input/js0`).
# 3. Run the script to initialize the robot and start listening for joystick inputs.
# 4. Use the joystick buttons to control the robot:
#    - "Start" button toggles the program's running state.
#    - "Home" button exits the program.
#    - "Button Y", "Button B", "Button A", and "Button X" control movement (forward, right, backward, left).
#    - "L1" and "R1" control rotation (left and right).
#    - "L2" and "R2" switch between servo modes 1 and 2.
# 5. Use the joystick axes to adjust servo positions in the selected servo mode:
#    - Servo mode 1: Adjusts servo 0 (X-Axis) and servo 1 (Y-Axis).
#    - Servo mode 2: Adjusts servo 2 (X-Axis) and servo 3 (Y-Axis).
# 6. Press "Home" or use Ctrl+C to safely exit the program.
# 7. Upon exit, the robot's velocity and joint positions are reset to their initial states.

import struct
import merlin_hw

# Initialize the robot
merlin_bot = merlin_hw.robot()
merlin_bot.start()

# Set the initial position of the robot's arm
merlin_bot.set_joint_pos(joint0_val=0, joint1_val=0, joint2_val=89, joint3_val=0)

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

# Mapping for axes
axis_map = {
    0: "X-Axis",
    1: "Y-Axis",
}

# Variables
servo_positions = [0, 0, 89, 0]  # Initial positions for four servos
servo_mode = None  # None, 1, or 2
program_running = False  # Toggle for Start button
exit_program = False  # Toggle for Home button

# Function to handle button commands
def handle_button_command(button_name, state):
    global servo_mode, program_running, exit_program

    if state == "Pressed":
        if button_name == "Start":
            program_running = not program_running
            print("Program started" if program_running else "Program stopped")

        elif button_name == "Home":
            exit_program = True
            print("Exiting program...")
            exit()

        elif button_name == "Select":
            servo_positions = [0, 0, 89, 0]
            merlin_bot.set_joint_pos(joint0_val=servo_positions[0], joint1_val=servo_positions[1],
                         joint2_val=servo_positions[2], joint3_val=servo_positions[3])
            print("Servos reset to original position")

        elif program_running:
            if button_name == "Button Y":
                print("Moving forward")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=-1, left_right_throttle=0, rotate_throttle=0)

            elif button_name == "Button B":
                print("Moving right")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=1, rotate_throttle=0)

            elif button_name == "Button A":
                print("Moving backward")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=1, left_right_throttle=0, rotate_throttle=0)

            elif button_name == "Button X":
                print("Moving left")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=-1, rotate_throttle=0)

            elif button_name == "L1":
                print("Rotating left")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=1)

            elif button_name == "R1":
                print("Rotating right")
                merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=-1)

            elif button_name == "L2":
                servo_mode = 1
                print("Servo mode 1 selected")

            elif button_name == "R2":
                servo_mode = 2
                print("Servo mode 2 selected")

    elif state == "Released":
        if button_name in ["Button Y", "Button B", "Button A", "Button X", "L1", "R1"]:
            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)

# Function to handle axis commands
def handle_axis_command(axis_name, value):
    global servo_positions, servo_mode

    if servo_mode == 1:
        if axis_name == "X-Axis":
            servo_positions[0] = max(-89, min(89, servo_positions[0] + (5 if value > 0 else -5)))
            print(f"Servo 0 position: {servo_positions[0]}")
        elif axis_name == "Y-Axis":
            servo_positions[1] = max(-89, min(89, servo_positions[1] + (5 if value > 0 else -5)))
            print(f"Servo 1 position: {servo_positions[1]}")

    elif servo_mode == 2:
        if axis_name == "X-Axis":
            servo_positions[2] = max(-89, min(89, servo_positions[2] + (5 if value > 0 else -5)))
            print(f"Servo 2 position: {servo_positions[2]}")
        elif axis_name == "Y-Axis":
            servo_positions[3] = max(-89, min(89, servo_positions[3] + (5 if value > 0 else -5)))
            print(f"Servo 3 position: {servo_positions[3]}")

    merlin_bot.set_joint_pos(joint0_val=servo_positions[0], joint1_val=servo_positions[1],
                             joint2_val=servo_positions[2], joint3_val=servo_positions[3])

# Main loop
try:
    while not exit_program:
        event = joystick.read(EVENT_SIZE)
        if event:
            time_val, value, event_type, number = struct.unpack(EVENT_FORMAT, event)

            if event_type == 1:  # Button event
                button_name = button_map.get(number, f"Unknown Button {number}")
                state = "Pressed" if value == 1 else "Released"
                handle_button_command(button_name, state)

            elif event_type == 2:  # Axis event
                axis_name = axis_map.get(number, f"Unknown Axis {number}")
                if program_running and servo_mode:
                    handle_axis_command(axis_name, value)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    joystick.close()
    merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)
    merlin_bot.set_joint_pos(joint0_val=0, joint1_val=0, joint2_val=89, joint3_val=0)
