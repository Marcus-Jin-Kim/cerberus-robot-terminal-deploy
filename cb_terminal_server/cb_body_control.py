from cb_low_level_control import CBLowLevelControl
import time

class CBBodyControl():

    def __init__(self, config, base_control_low:CBLowLevelControl): #, robot_type="WS_UGV_BEAST_PI5"):
        self.config = config
        # self.robot_type = robot_type
        self.base = base_control_low

        self.default_linear_speed = self.config["BODY_DEFAULT_LINEAR_SPEED"]  # 0.2  # Default speed in m/s
        self.default_turn_speed = self.config["BODY_DEFAULT_TURN_SPEED"]
        self.default_duration = self.config["BODY_DEFAULT_MOVE_DURATION"] # 0.5  # Default duration in seconds

    def direct_speed_control(self, vspeed, aspeed, duration):
        # duration 0 means continuous until stop is called
        if duration < 0:
            duration = self.default_duration

        self.base.send_command({"T": 1, "L": vspeed+aspeed, "R": vspeed-aspeed})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.base.send_command({"T": 1, "L": 0, "R": 0})    



    # def _forward(self, vspeed, duration):

    #     if vspeed <= 0:
    #         vspeed = self.default_linear_speed
    #     # duration 0 means continuous until stop is called
    #     if duration < 0:
    #         duration = self.default_duration

    #     self.base.send_command({"T": 1, "L": vspeed, "R": vspeed})

    #     if (duration > 0):
    #         # Wait for the specified duration
    #         time.sleep(duration)
    #         self.base.send_command({"T": 1, "L": 0, "R": 0})
        
    # def _back(self, vspeed, duration):
    #     if vspeed <= 0:
    #         vspeed = self.default_linear_speed
    #     # duration 0 means continuous until stop is called
    #     if duration < 0:
    #         duration = self.default_duration

    #     self.base.send_command({"T": 1, "L": -vspeed, "R": -vspeed})

    #     if (duration > 0):
    #         # Wait for the specified duration
    #         time.sleep(duration)
    #         self.base.send_command({"T": 1, "L": 0, "R": 0})    

    # def stop(self):
    #     """
    #     Stop the robot.
    #     """
    #     self.base.send_command({"T": 1, "L": 0, "R": 0})
    
    # def _moveleft(self, speed=0.2, duration=0.5):
    #     """
    #     Move the robot to the left at a specified speed for a given duration.
    #     :param speed: Speed in meters per second.
    #     :param duration: Duration in seconds.
    #     """
    #     # UGV beast cannot move left, so we will just stop the right wheel
    #     self.stop()
        
    # def _moveright(self, speed=0.2, duration=0.5):
    #     """
    #     Move the robot to the right at a specified speed for a given duration.
    #     :param speed: Speed in meters per second.
    #     :param duration: Duration in seconds.
    #     """
    #     # UGV beast cannot move right, so we will just stop the left wheel
    #     self.stop()

    # def _turnleft(self, vspeed, aspeed, duration):
    #     # duration 0 means continuous until stop is called
    #     if duration < 0:
    #         duration = self.default_duration

    #     self.base.send_command({"T": 1, "L": vspeed-aspeed, "R": vspeed+aspeed})

    #     if (duration > 0):
    #         # Wait for the specified duration
    #         time.sleep(duration)
    #         self.base.send_command({"T": 1, "L": 0, "R": 0})
    
    # def _turnright(self, vspeed, aspeed, duration):
    #     # duration 0 means continuous until stop is called
    #     if duration < 0:
    #         duration = self.default_duration

    #     self.base.send_command({"T": 1, "L": vspeed+aspeed, "R": vspeed-aspeed})

    #     if (duration > 0):
    #         # Wait for the specified duration
    #         time.sleep(duration)
    #         self.base.send_command({"T": 1, "L": 0, "R": 0})    

