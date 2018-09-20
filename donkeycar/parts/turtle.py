class RouteWriter:
    def __init__(self):
        self.route = []

    def forward(self, throttle, ticks):
        """Set car's throttle for given amount of ticks"""
        self.route.extend([{'type': 'throttle', 'value': throttle} for i in range(ticks)])

    def turn(self, angle):
        """Set car's angle for a given amount of ticks"""
        self.route.append({'type': 'steering', 'value': angle})

    def generate(self):
        x = self.route
        self.route = []
        return x

class RouteReader:

    def __init__(self, route):
        self.route = route
        self.current_index = 0
        self.prev_angle = 0
        self.prev_throttle = 0

    def run(self):
        try:
            data = self.route[self.current_index]

            while data['type'] == 'steering':
                angle = data['value']
                throttle = self.prev_throttle
                self.prev_angle = angle

                self.current_index += 1
                data = self.route[self.current_index]

            throttle = data['value']
            self.prev_throttle = throttle
            angle = self.prev_angle

            self.current_index += 1
            return angle, throttle
        except IndexError:
            return 0, 0
