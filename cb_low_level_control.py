from base_ctrl import BaseController
import time

_SERIAL_PORT_ = '/dev/ttyAMA0' # UGV_BEAST_PI5
_BAUD_RATE_ = 115200


class CBLowLevelControl():


    def __init__(self):
        self.base_control_low = BaseController(_SERIAL_PORT_, _BAUD_RATE_)
        self.base_control_low.send_command({"T": 900, "main": 3, "module": 2}) # UGV Beast PI5

    def send_command(self, command):
        self.base_control_low.send_command(command)

    def move_forward(self, speed, duration):
        self.base_control_low.send_command({"T": 1, "L": speed, "R": speed})
        if duration > 0:
            time.sleep(duration) # FIXME: blocks the whole robot
            self.base_control_low.send_command({"T": 1, "L": 0, "R": 0})

    def set_turret_pan_tilt(self, pan, tilt):
        pass
        # self.base_control.send_command({"T": 900, "main": 3, "module": 2, "pan": pan, "tilt": tilt})

    # def set_speed(self, left_speed, right_speed):
    #     self.base_control.set_speed(left_speed, right_speed)

    # def stop(self):
    #     self.base_control.stop()

    # def cleanup(self):
    #     self.base_control.cleanup()