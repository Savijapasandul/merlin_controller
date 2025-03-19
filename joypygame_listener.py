import pygame

pygame.init()
pygame.joystick.init()

# Check if any joysticks are connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected.")
    pygame.quit()
    exit()
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Joystick Name: {joystick.get_name()}")

    try:
        while True:
            pygame.event.pump()  # Update internal state of the joystick

            # Get button states
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

            # Get axis values (analog input)
            axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]

            # Print the joystick axes and button states
            print(f"Axes: {axes}")
            print(f"Buttons: {buttons}")

            pygame.time.wait(100)  # Prevent overwhelming the output
    except KeyboardInterrupt:
        print("\nExiting...")
        pygame.quit()
