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

# Mapping for axes and directions
axis_map = {
    0: "X-Axis (Left/Right)",
    1: "Y-Axis (Forward/Backward)",
}

# Variables to store servo positions
servo_positions = [0, 0, 89, 0]  # Initial positions for four servos
stationary_mode = False  # Flag to indicate stationary mode
selected_servo = 0  # Index of the currently selected servo (0 to 3)

# Function to handle button commands
def handle_button_command(button_name, state):
    global servo_positions, stationary_mode, selected_servo

    if state == "Pressed":
        if button_name == "Button Y":
            print("Button Y: Entering stationary mode")
            stationary_mode = True
            # Stop the robot's movement
            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)
        
        elif button_name == "L2":
            if stationary_mode:
                # Decrease the selected servo's position by 10 (clamped to -89)
                servo_positions[selected_servo] = max(-89, servo_positions[selected_servo] - 10)
                print(f"L2: Servo {selected_servo} position decreased to {servo_positions[selected_servo]}")
                # Update the servo motor positions
                merlin_bot.set_joint_pos(joint0_val=servo_positions[0], joint1_val=servo_positions[1], joint2_val=servo_positions[2], joint3_val=servo_positions[3])
        
        elif button_name == "R2":
            if stationary_mode:
                # Increase the selected servo's position by 10 (clamped to 89)
                servo_positions[selected_servo] = min(89, servo_positions[selected_servo] + 10)
                print(f"R2: Servo {selected_servo} position increased to {servo_positions[selected_servo]}")
                # Update the servo motor positions
                merlin_bot.set_joint_pos(joint0_val=servo_positions[0], joint1_val=servo_positions[1], joint2_val=servo_positions[2], joint3_val=servo_positions[3])
        
        elif button_name == "Button B":
            print("Button B: Exiting stationary mode")
            stationary_mode = False
            # Reset all servo positions to 0
            servo_positions = [0, 0, 89, 0]
            merlin_bot.set_joint_pos(joint0_val=0, joint1_val=0, joint2_val=89, joint3_val=0)
        
        elif button_name == "Button X":
            if stationary_mode:
                # Cycle to the next servo
                selected_servo = (selected_servo + 1) % 4
                print(f"Button X: Selected Servo {selected_servo}")
        
        elif button_name == "Button A":
            if stationary_mode:
                # Cycle to the previous servo
                selected_servo = (selected_servo - 1) % 4
                print(f"Button A: Selected Servo {selected_servo}")
        
        elif button_name == "L1":
            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=1)
        
        elif button_name == "R1":
            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=-1)

        else:
            print(f"{button_name} pressed, but no action defined")
    
    elif state == "Released":
        print(f"{button_name} released")
        merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)

try:
    while True:
        event = joystick.read(EVENT_SIZE)
        if event:
            (time_val, value, event_type, number) = struct.unpack(EVENT_FORMAT, event)
            if event_type == 1:  # Button event
                button_name = button_map.get(number, f"Unknown Button {number}")
                state = "Pressed" if value == 1 else "Released"
                print(f"Time: {time_val}, {button_name} {state}")
                handle_button_command(button_name, state)
            elif event_type == 2:  # Axis event
                axis_name = axis_map.get(number, f"Unknown Axis {number}")
                if not stationary_mode:  # Only process axis events if not in stationary mode
                    if number == 0:  # X-Axis (Left/Right)
                        if value == -32767:
                            print(f"Time: {time_val}, Left")
                            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=-1, rotate_throttle=0)
                        elif value == 32767:
                            print(f"Time: {time_val}, Right")
                            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=1, rotate_throttle=0)
                        elif value == 0:
                            print(f"Time: {time_val}, X-Axis Centered")
                            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)
                    elif number == 1:  # Y-Axis (Forward/Backward)
                        if value == -32767:
                            print(f"Time: {time_val}, Forward")
                            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=-1, left_right_throttle=0, rotate_throttle=0)
                        elif value == 32767:
                            print(f"Time: {time_val}, Backward")
                            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=1, left_right_throttle=0, rotate_throttle=0)
                        elif value == 0:
                            print(f"Time: {time_val}, Y-Axis Centered")
                            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)                     
                    # elif number in (0, 1):  # Handle diagonal movement
                    #     if axis_map.get(0) and axis_map.get(1):
                    #         if value == -32767:
                    #             if number == 0:  # Left
                    #                 print(f"Time: {time_val}, Diagonal Forward-Left")
                    #                 merlin_bot.set_velocity_throttle(fwd_bwd_throttle=-1, left_right_throttle=-1, rotate_throttle=0)
                    #             elif number == 1:  # Forward
                    #                 print(f"Time: {time_val}, Diagonal Forward-Left")
                    #                 merlin_bot.set_velocity_throttle(fwd_bwd_throttle=-1, left_right_throttle=-1, rotate_throttle=0)
                    #         elif value == 32767:
                    #             if number == 0:  # Right
                    #                 print(f"Time: {time_val}, Diagonal Forward-Right")
                    #                 merlin_bot.set_velocity_throttle(fwd_bwd_throttle=-1, left_right_throttle=1, rotate_throttle=0)
                    #             elif number == 1:  # Backward
                    #                 print(f"Time: {time_val}, Diagonal Backward-Right")
                    #                 merlin_bot.set_velocity_throttle(fwd_bwd_throttle=1, left_right_throttle=1, rotate_throttle=0)
                    #         elif value == 0:
                    #             print(f"Time: {time_val}, Diagonal Centered")
                    #             merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    joystick.close()
    merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)
    merlin_bot.set_joint_pos(joint0_val=0, joint1_val=0, joint2_val=89, joint3_val=0)