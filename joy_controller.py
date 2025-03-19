import struct
import merlin_hw

# Initialize the robot
merlin_bot = merlin_hw.robot()
merlin_bot.start()

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

# Function to handle button commands
def handle_button_command(button_name, state):
    if state == "Pressed":
        if button_name == "Button A":
            print("Button A: Starting robot")
            
        elif button_name == "Button B":
            print("Button B: Stopping robot")
            
        elif button_name == "Button X":
            print("Button X: Rotating left")
            
        elif button_name == "Button Y":
            print("Button Y: Rotating right")
            
        elif button_name == "L1":
            print("L1: Rotating left")
            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=1)
        elif button_name == "R1":
            print("R1: Rotating right")
            merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=-1)
        elif button_name == "Start":
            print("Start: Resetting robot")
            
        elif button_name == "Select":
            print("Select: Toggle mode")
            
        else:
            print(f"{button_name} pressed, but no action defined")

try:
    while True:
        # Read an event from the joystick
        event = joystick.read(EVENT_SIZE)
        if event:
            # Unpack the event data
            (time_val, value, event_type, number) = struct.unpack(EVENT_FORMAT, event)

            # Handle button events
            if event_type == 1:
                button_name = button_map.get(number, f"Unknown Button {number}")
                state = "Pressed" if value == 1 else "Released"
                print(f"Time: {time_val}, {button_name} {state}")
                handle_button_command(button_name, state)

            # Handle axis events
            elif event_type == 2:
                axis_name = axis_map.get(number, f"Unknown Axis {number}")

                if number == 0:  # X-Axis (Left/Right)
                    if value == -32767:
                        print(f"Time: {time_val}, Left")
                        merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=1, rotate_throttle=0)
                    elif value == 32767:
                        print(f"Time: {time_val}, Right")
                        merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=-1, rotate_throttle=0)
                    elif value == 0:
                        print(f"Time: {time_val}, X-Axis Centered")
                        merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)

                elif number == 1:  # Y-Axis (Forward/Backward)
                    if value == -32767:
                        print(f"Time: {time_val}, Forward")
                        merlin_bot.set_velocity_throttle(fwd_bwd_throttle=1, left_right_throttle=0, rotate_throttle=0)
                    elif value == 32767:
                        print(f"Time: {time_val}, Backward")
                        merlin_bot.set_velocity_throttle(fwd_bwd_throttle=-1, left_right_throttle=0, rotate_throttle=0)
                    elif value == 0:
                        print(f"Time: {time_val}, Y-Axis Centered")
                        merlin_bot.set_velocity_throttle(fwd_bwd_throttle=0, left_right_throttle=0, rotate_throttle=0)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    # Close the joystick device
    joystick.close()