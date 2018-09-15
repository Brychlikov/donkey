from donkeycar.parts.controller import Joystick
import json

j = Joystick('/dev/input/js0')
j.init()
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

        print(button, state, axis, val)

        if button and state:
            choice = input("what function to assign? Enter name or list index or 'l' to print the list")
            try:
                int_choice = int(choice)
            except:
                int_choice = None
            while not (int_choice in range(len(button_options)) or choice in button_options):
                if choice == 'l':
                    print("Possible functions:")
                    for i, entry in enumerate(button_options):
                        print('{}. {}'.format(i, entry))
                    choice = input("Choose an option")
                else:
                    choice = input("Wrong option. Enter again")

                try:
                    int_choice = int(choice)
                except:
                    int_choice = None

            if len(choice) == 1:
                choice = button_options[int(choice)]

            button_fun_map[button] = choice

        elif axis and (val == 1.0 or val == -1.0):
            choice = input("what function to assign? Enter name or list index or 'l' to print the list")

            try:
                int_choice = int(choice)
            except:
                int_choice = None

            while not (int_choice in range(len(axis_options)) or choice in axis_options):
                if choice == 'l':
                    print("Possible functions:")
                    for i, entry in enumerate(axis_options):
                        print('{}. {}'.format(i, entry))
                    choice = input("Specify an option")
                else:
                    choice = input("Wrong option. Enter again")

                try:
                    int_choice = int(choice)
                except:
                    int_choice = None

            if len(choice) == 1:
                choice = axis_options[int(choice)]
            axis_fun_map[axis] = choice
except KeyboardInterrupt:
    print(button_fun_map)
    print(axis_fun_map)
    with open('output.json', 'w') as f:
        json.dump([button_fun_map, axis_fun_map], f)


