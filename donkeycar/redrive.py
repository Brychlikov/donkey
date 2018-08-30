from argparse import ArgumentParser
from donkeycar.parts.datastore import Tub, TubReader
from donkeycar import Vehicle, load_config
from donkeycar.parts.actuator import RaspberryPi_PWM, PWMSteering, PWMThrottle

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-p', '--path', help='Specify car path to replay')
    args = parser.parse_args()

    cfg = load_config(args.path)
    v = Vehicle()
    t = TubReader(path=args.path)
    v.add(t, inputs=['data_to_read'], outputs=['angle', 'throttle'])
    v.mem['data_to_read'] = ['user/angle', 'user/throttle']

    steering_controller = RaspberryPi_PWM(cfg.STEERING_CHANNEL)
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=cfg.STEERING_LEFT_PWM,
                           right_pulse=cfg.STEERING_RIGHT_PWM)

    throttle_controller = RaspberryPi_PWM(cfg.THROTTLE_CHANNEL)
    throttle = PWMThrottle(controller=throttle_controller,
                           max_pulse=cfg.THROTTLE_FORWARD_PWM,
                           zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                           min_pulse=cfg.THROTTLE_REVERSE_PWM)

    v.add(steering, inputs=['angle'])
    v.add(throttle, inputs=['throttle'])

    v.start(rate_hz=cfg.DRIVE_LOOP_HZ)

