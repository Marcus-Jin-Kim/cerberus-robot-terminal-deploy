from cb_low_level_control import CBLowLevelControl
import time

class CBBodyControl():

    def __init__(self, config, base_control_low:CBLowLevelControl): #, robot_type="WS_UGV_BEAST_PI5"):
        self.config = config
        # self.robot_type = robot_type
        self.base = base_control_low

        self.default_linear_speed_slow = self.config["BODY_DEFAULT_LINEAR_SPEED_SLOW"]  # 0.2  # Default speed in m/s
        self.default_linear_speed_cruise = self.config["BODY_DEFAULT_LINEAR_SPEED_CRUISE"]
        self.default_linear_speed_fast = self.config["BODY_DEFAULT_LINEAR_SPEED_FAST"]
        self.default_turn_speed = self.config["BODY_DEFAULT_TURN_SPEED"]        
        # self.default_duration = self.config["BODY_DEFAULT_MOVE_DURATION"] # 0.5  # Default duration in seconds
        # 
        # CMD_HEART_BEAT_SET
        # {"T":136,"cmd":3000}
        # Sets the Heartbeat Function Interval.
        # The cmd unit is milliseconds. 
        # This command sets the interval for the heartbeat function. 
        # If the sub-controller does not receive a new motion command within this time, 
        # it will automatically stop movement. 
        # This feature helps prevent continuous movement in case the host crashes.
        # means MAX_DURATION = 3



    def stop(self):
        """
        Stop the robot.
        """
        self.base.send_command({"T": 1, "L": 0, "R": 0})    


    def direct_speed_control(self, vspeed, aspeed, duration):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        self.base.send_command({"T": 1, "L": vspeed+aspeed, "R": vspeed-aspeed})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.base.send_command({"T": 1, "L": 0, "R": 0})    


    def forward_slow(self, duration):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        self.base.send_command({"T": 1, "L": self.default_linear_speed_slow, 
                                "R": self.default_linear_speed_slow})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.stop()

    def forward_cruise(self, duration):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        self.base.send_command({"T": 1, "L": self.default_linear_speed_cruise, 
                                "R": self.default_linear_speed_cruise})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.stop()
    
    def forward_fast(self, duration):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        self.base.send_command({"T": 1, "L": self.default_linear_speed_fast, 
                                "R": self.default_linear_speed_fast})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.stop()
    
    def back_slow(self, duration):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        self.base.send_command({"T": 1, "L": -self.default_linear_speed_slow, 
                                "R": -self.default_linear_speed_slow})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.stop()

    def back_cruise(self, duration):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        self.base.send_command({"T": 1, "L": -self.default_linear_speed_cruise, 
                                "R": -self.default_linear_speed_cruise})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.stop()
    
    def back_fast(self, duration):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        self.base.send_command({"T": 1, "L": -self.default_linear_speed_fast, 
                                "R": -self.default_linear_speed_fast})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.stop()

    def turnleft(self, duration, reverse=False):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        if reverse:
            self.base.send_command({"T": 1, 
                                    "L": -self.default_linear_speed_cruise - self.default_turn_speed, 
                                    "R": -self.default_linear_speed_cruise + self.default_turn_speed})
        else:
            self.base.send_command({"T": 1, 
                                    "L": self.default_linear_speed_cruise - self.default_turn_speed, 
                                    "R": self.default_linear_speed_cruise + self.default_turn_speed})            

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.stop()

    def turnright(self, duration, reverse=False):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        if reverse:
            self.base.send_command({"T": 1, 
                                    "L": -self.default_linear_speed_cruise + self.default_turn_speed, 
                                    "R": -self.default_linear_speed_cruise - self.default_turn_speed})
        else:
            self.base.send_command({"T": 1, 
                                    "L": self.default_linear_speed_cruise + self.default_turn_speed, 
                                    "R": self.default_linear_speed_cruise - self.default_turn_speed})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.stop()


    def pivotleft(self, duration):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        self.base.send_command({"T": 1, 
                                "L": -self.default_turn_speed, 
                                "R": self.default_turn_speed})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.stop()

    def pivotright(self, duration):
        # default duration is controlled by Unity, not here (we have heartbeat anyway)
        # # duration 0 means continuous until stop is called
        # if duration < 0:
        #     duration = self.default_duration

        self.base.send_command({"T": 1, 
                                "L": self.default_turn_speed, 
                                "R": -self.default_turn_speed})

        if (duration > 0):
            # Wait for the specified duration
            time.sleep(duration)
            self.stop()
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

