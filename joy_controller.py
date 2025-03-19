from evdev import InputDevice, categorize, ecodes

# Replace with your gamepad's event path
device_path = "/dev/input/event0"

# Map button codes to their names
button_names = {
    ecodes.BTN_SOUTH: "South Button (A)",
    ecodes.BTN_EAST: "East Button (B)",
    ecodes.BTN_NORTH: "North Button (Y)",
    ecodes.BTN_WEST: "West Button (X)",
    ecodes.BTN_TL: "Left Bumper (LB)",
    ecodes.BTN_TR: "Right Bumper (RB)",
    ecodes.BTN_TL2: "Left Trigger (LT)",
    ecodes.BTN_TR2: "Right Trigger (RT)",
    ecodes.BTN_SELECT: "Select Button",
    ecodes.BTN_START: "Start Button",
    ecodes.BTN_MODE: "Mode Button",
    ecodes.BTN_THUMBL: "Left Thumbstick Button",
    ecodes.BTN_THUMBR: "Right Thumbstick Button",
}

# Map axis codes to their names
axis_names = {
    ecodes.ABS_X: "Left Stick X-Axis",
    ecodes.ABS_Y: "Left Stick Y-Axis",
    ecodes.ABS_RX: "Right Stick X-Axis",
    ecodes.ABS_RY: "Right Stick Y-Axis",
    ecodes.ABS_Z: "Left Trigger (Analog)",
    ecodes.ABS_RZ: "Right Trigger (Analog)",
    ecodes.ABS_HAT0X: "D-Pad X-Axis",
    ecodes.ABS_HAT0Y: "D-Pad Y-Axis",
}

try:
    # Initialize the gamepad device
    gamepad = InputDevice(device_path)
    print(f"Using gamepad: {gamepad.name}")

    # Main loop to read events
    for event in gamepad.read_loop():
        if event.type == ecodes.EV_KEY:  # Button press/release
            button_name = button_names.get(event.code, f"Unknown Button (Code: {event.code})")
            print(f"Button: {button_name}, State: {'Pressed' if event.value else 'Released'}")

        elif event.type == ecodes.EV_ABS:  # Axis movement
            axis_name = axis_names.get(event.code, f"Unknown Axis (Code: {event.code})")
            print(f"Axis: {axis_name}, Value: {event.value}")

except Exception as e:
    print(f"An error occurred: {e}")