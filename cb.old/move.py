from base_ctrl import BaseController
import time


class Move:
    def __init__(self, robot_type="WS_UGV_BEAST_PI5"):
        self.robot_type = robot_type
        # TODO : read from some master type config file
        self.base = BaseController('/dev/ttyAMA0', 115200) 
        self.default_linear_speed = 0.2  # Default speed in m/s
        self.default_turn_speed = 0.2
        self.default_duration = 0.5  # Default duration in seconds

    def forward(self, speed, duration):

        if speed <= 0:
            speed = self.default_linear_speed
        if duration <= 0:
            duration = self.default_duration

        self.base.send_command({"T": 1, "L": speed, "R": speed})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.base.send_command({"T": 1, "L": 0, "R": 0})
        
    def back(self, speed, duration):
        if speed <= 0:
            speed = self.default_linear_speed
        if duration <= 0:
            duration = self.default_duration

        self.base.send_command({"T": 1, "L": -speed, "R": -speed})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.base.send_command({"T": 1, "L": 0, "R": 0})    

    def stop(self):
        """
        Stop the robot.
        """
        self.base.send_command({"T": 1, "L": 0, "R": 0})
    
    def moveleft(self, speed=0.2, duration=0.5):
        """
        Move the robot to the left at a specified speed for a given duration.
        :param speed: Speed in meters per second.
        :param duration: Duration in seconds.
        """
        # UGV beast cannot move left, so we will just stop the right wheel
        self.stop()
        
    def moveright(self, speed=0.2, duration=0.5):
        """
        Move the robot to the right at a specified speed for a given duration.
        :param speed: Speed in meters per second.
        :param duration: Duration in seconds.
        """
        # UGV beast cannot move right, so we will just stop the left wheel
        self.stop()

    def turnleft(self, speed, duration):
        if speed <= 0:
            speed = self.default_turn_speed
        if duration <= 0:
            duration = self.default_duration

        self.base.send_command({"T": 1, "L": -speed, "R": speed})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.base.send_command({"T": 1, "L": 0, "R": 0})
    
    def turnright(self, speed, duration):
        if speed <= 0:
            speed = self.default_turn_speed
        if duration <= 0:
            duration = self.default_duration

        self.base.send_command({"T": 1, "L": speed, "R": -speed})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.base.send_command({"T": 1, "L": 0, "R": 0})    