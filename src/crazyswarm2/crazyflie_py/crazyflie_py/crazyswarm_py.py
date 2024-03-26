import rclpy

from . import genericJoystick
from .crazyflie import CrazyflieServer, TimeHelper


class Crazyswarm:

    def __init__(self):
        try:
            rclpy.init()
        except Exception as e:
            print(e)

        print("here1")
        self.allcfs = CrazyflieServer()
        print("here2")
        self.timeHelper = TimeHelper(self.allcfs)

        # self.input = genericJoystick.Joystick(self.timeHelper)
