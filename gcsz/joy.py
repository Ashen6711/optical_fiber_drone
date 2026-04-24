import time
import sdl2
import os
os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/event22"

if sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK) != 0:
    raise RuntimeError("SDL init failed")

count = sdl2.SDL_NumJoysticks()

print(f"Joysticks found: {count}")

if count == 0:
    raise SystemExit("No joystick connected")

for i in range(count):
    name = sdl2.SDL_JoystickNameForIndex(i)
    if name:
        print(f"{i}: {name.decode()}")
    else:
        print(f"{i}: Unknown")

JOY_INDEX = 0

joy = sdl2.SDL_JoystickOpen(JOY_INDEX)

if not joy:
    raise RuntimeError(f"Could not open joystick {JOY_INDEX}")

print("\nOpened:", sdl2.SDL_JoystickName(joy).decode())

num_axes = sdl2.SDL_JoystickNumAxes(joy)
num_buttons = sdl2.SDL_JoystickNumButtons(joy)

prev_axes = [None] * num_axes
prev_buttons = [None] * num_buttons

print(f"Axes: {num_axes}")
print(f"Buttons: {num_buttons}")
print()

try:
    while True:
        sdl2.SDL_JoystickUpdate()

        axis_values = []
        for axis in range(num_axes):
            raw = sdl2.SDL_JoystickGetAxis(joy, axis)
            if raw != prev_axes[axis]:
                print(f"Axis {axis}: {raw}")
                prev_axes[axis] = raw

        pressed_buttons = []
        for button in range(num_buttons):
            raw = sdl2.SDL_JoystickGetButton(joy, button)
            if raw != prev_buttons[button]:
                print(f"Button {button}: {raw}")
                prev_buttons[button] = raw

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting")

finally:
    sdl2.SDL_JoystickClose(joy)
    sdl2.SDL_Quit()