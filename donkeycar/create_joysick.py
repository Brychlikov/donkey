from donkeycar.parts.controller import Joystick
import json

j = Joystick()
button_fun_map = {}
axis_fun_map = {}
button_options = [
    "change_mode",
    "record_toggle",
    "max_throttle_increase",
    "max_throttle_decrease",
    "throttle_scale_increase",
    "throttle_scale_decrease",
    "steering_scale_decrease",
    "steering_scale_increase",
    "constant_throttle_toggle"
]

axis_options = [
    "throttle",
    "steering"
]
try:
    while True:
        print('Press button you want to set or bend axis')
        button, state, axis, val = j.poll()
        while not state and val != 1.0 and val != -1.0:
            button, state, axis, val = j.poll()

        if button and state:
            choice = input("what function to assign? Enter name or list index or 'l' to print the list")
            while not (int(choice) in range(len(button_options)) or choice in button_options):
                if choice == 'l':
                    print("Possible functions:")
                    for i, entry in enumerate(button_options):
                        print('{}. {}'.format(i, entry))
                else:
                    choice = input("Wrong option. Enter again")
            if len(choice) == 1:
                choice = button_options[int(choice)]

            button_fun_map[button] = choice

        elif axis and (val == 1.0 or val == -1.0):
            choice = input("what function to assign? Enter name or list index or 'l' to print the list")
            while not (int(choice) in range(len(axis_options)) or choice in axis_options):
                if choice == 'l':
                    print("Possible functions:")
                    for i, entry in enumerate(axis_options):
                        print('{}. {}'.format(i, entry))
                else:
                    choice = input("Wrong option. Enter again")
            if len(choice) == 1:
                choice = axis_options[int(choice)]
except KeyboardInterrupt:
    print(button_fun_map)
    print(axis_fun_map)
    with file('output.json', 'w') as f:
        json.dump([button_fun_map, axis_fun_map], f)



# Functions:

# axis:
# -throttle
# -angle

# buttons:
# -change_mode
# -record_toggle
# -max_throttle_increase
# -max_throttle_decrease
# -throttle_scale_increase
# -throttle_scale_decrease
# -steering_scale_decrease
# -steering_scale_increase
# -constant_throttle_toggle
