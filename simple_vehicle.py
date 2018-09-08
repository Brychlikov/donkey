from donkeycar import Vehicle, load_config
from donkeycar.parts.actuator import RaspberryPi_PWM, PWMSteering, PWMThrottle
from donkeycar.parts.camera  import PiCamera
from donkeycar.parts.pilots import LineFollower
from donkeycar.parts.transform import Lambda
from donkeycar.parts.datastore import Tub, TubWriter
import time


def crop_image(img, l, n=1):
    """extracts top l (every other n) lines from image array"""

    return img[:l:n, :, :]

v = Vehicle()
cfg = load_config('/home/pi/fork_car/config.py')

cam = PiCamera(resolution=cfg.CAMERA_RESOLUTION)
v.add(cam, outputs=['raw_img_array'], threaded=True)

v.mem['const_ines'] = 40
v.mem['const_every_other'] = 5
v.add(Lambda(crop_image), inputs=['raw_img_array', 'const_lines', 'const_every_other'], outputs=['img_array'])

v.add(LineFollower(), inputs=['img_array'], outputs=['angle', 'found_lines'])

steering_controller = PCA9685(cfg.STEERING_CHANNEL) if cfg.PWM_CONTROLLER == 'PCA9685' else RaspberryPi_PWM(cfg.STEERING_CHANNEL)
steering = PWMSteering(controller=steering_controller,
                      left_pulse=cfg.STEERING_LEFT_PWM,
                      right_pulse=cfg.STEERING_RIGHT_PWM)


throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL) if cfg.PWM_CONTROLLER == 'PCA9685' else RaspberryPi_PWM(cfg.THROTTLE_CHANNEL)
throttle = PWMThrottle(controller=throttle_controller,
                       max_pulse=cfg.THROTTLE_FORWARD_PWM,
                       zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                       min_pulse=cfg.THROTTLE_REVERSE_PWM)

v.add(steering, inputs=['angle'])
v.add(throttle, inputs=['const'])
v.mem['const'] = 0.51

inputs = ['img_array', 'angle', 'throttle', 'found_lines']
types = ['image_array', 'float', 'float', 'str']

t = TubWriter(path=cfg.TUB_PATH, inputs=inputs, types=types)
v.add(t, inputs=inputs)

v.start(rate_hz = cfg.DRIVE_LOOP_HZ)
