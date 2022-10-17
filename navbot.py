import math
from collections import namedtuple
from time import sleep

import gps
from RpiMotorLib.rpi_dc_lib import L298NMDc

coordinates = namedtuple("coordinates", ["latitude", "longitude"])


class Steering:
    def __init__(self, right_motor, left_motor):
        self.right_motor = right_motor
        self.left_motor = left_motor

    def turn(self, duty_cycle=50):
        self.right_motor.forward(duty_cycle)
        self.left_motor.backward(duty_cycle)

    def forward(self, duty_cycle=50):
        self.right_motor.forward(duty_cycle)
        self.left_motor.forward(duty_cycle)

    def stop(self):
        self.right_motor.stop()
        self.left_motor.stop()

    def __del__(self):
        self.left_motor.cleanup(True)
        self.right_motor.cleanup(True)


class Navigation:
    def __init__(self):
        self.session = gps.gps(mode=gps.WATCH_ENABLE)

    def _get_fix(self):
        while 0 == self.session.read():
            if not (gps.MODE_SET & self.session.valid):
                continue

            if (
                gps.isfinite(self.session.fix.latitude)
                and gps.isfinite(self.session.fix.longitude)
                and gps.isfinite(self.session.fix.track)
            ):
                return
            else:
                print("Acquiring GPS fix...")
                sleep(0.5)
        raise RuntimeError("GPS error")

    @property
    def coordinates(self):
        self._get_fix()
        return coordinates(self.session.fix.latitude, self.session.fix.longitude)

    @property
    def track(self):
        self._get_fix()
        return (
            self.session.fix.track
        )  # Just a guess, I don't know how to get the track value.

    def __del__(self):
        self.session.close()


class Robot:
    def __init__(self, right_motor, left_motor):
        self.steering = Steering(right_motor, left_motor)
        self.navigation = Navigation()
        self.target = None

    def get_distance_and_bearing(self):
        if self.target:
            x1 = self.navigation.coordinates.latitude
            y1 = self.navigation.coordinates.longitude
            x2 = self.target.latitude
            y2 = self.target.longitude

            dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) * 100000
            bearing = math.atan2(y2 - y1, x2 - x1) * (180 / math.pi)
            bearing %= 360
            return dist, bearing
        else:
            print("No target")

    def drive_to(self, latitude, longitude):
        self.target = coordinates(latitude, longitude)
        distance, bearing = self.get_distance_and_bearing()
        while distance > 0.02:
            self._face_target()
            self.steering.forward()
            print(f"Driving toward target, {distance} remaining")
            sleep(10)  # Drive for 10 seconds, then course correct.
            # The drive time will probably need to be adjusted based on distance to target.
        self.steering.stop()
        print("You have arrived.")

    def _face_target(self):
        _, bearing = self.get_distance_and_bearing()
        while abs(self.navigation.track - bearing) > 5:
            print(f"Turning from {self.navigation.track} toward {bearing}")
            self.steering.turn(duty_cycle=20)
            sleep(0.05)
            _, bearing = self.get_distance_and_bearing()
        self.steering.stop()


left_motor = L298NMDc(6, 13, 18, name="left")
right_motor = L298NMDc(19, 12, 26, name="right")
robot = Robot(right_motor, left_motor)
target = (
    float(input("Enter target latitude: ")),
    float(input("Enter target longitude: ")),
)
try:
    robot.drive_to(*target)
except Exception as exc:
    raise exc
finally:
    robot.steering.stop()
