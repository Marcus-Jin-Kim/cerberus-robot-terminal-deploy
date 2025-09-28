from base_ctrl import BaseController
import time

# # Configuration for UGV Beast Pi5
# ROBOT_TYPE: 'UGV_BEAST_PI5'
# LOW_LEVEL_CONTROL_SERIAL_PORT: '/dev/ttyAMA0' # UGV_BEAST_PI5
# #LOW_LEVEL_CONTROL_SERIAL_PORT: '/dev/ttyTHS1' # UGV_ROVER_JETSON_(ORIN? NX?)
# LOW_LEVEL_CONTROL_BAUD_RATE: 115200


class CBLowLevelControl():


    def __init__(self, config):
        self.config = config
        self.has_base_control_low_initialized = False
        try:
            self.base_control_low = BaseController(config["LOW_LEVEL_CONTROL_SERIAL_PORT"], config["LOW_LEVEL_CONTROL_BAUD_RATE"])
            # maybe this makes it slow?
            # app.py does this differently.. for now put them in cb_terminal_server.py
            # self.base_control_low.send_command({"T": 900, "main": 3, "module": 2}) # UGV Beast PI5
            # setting module 2 in T 900 does give pan tilt feedback, but it's really slow
            # let's ignore module number for now (like app.py)
            self.base_control_low.send_command({"T":142,"cmd":50})   # set feedback interval
            self.base_control_low.send_command({"T":131,"cmd":1})    # serial feedback flow on:
            #################
            ## !!!! feedback flow MUST BE ON TO GET FEEDBACK DATA AND UPDATE ROS POSITION TF AND EVERYTHING
            ##################
            #             
            self.base_control_low.send_command({"T":143,"cmd":0})    # serial echo off ## MAYBE THIS IS IMPORTANT?
            self.base_control_low.send_command({"T":4,"cmd":0})     #                 
                # f['base_config']['module_type']}) # select the module - 0:None 1:RoArm-M2-S 2:Gimbal
                # 0 is hardcoded for ugv_rpi/config.yaml (from waveshare. wtf)
                # https://www.waveshare.com/wiki/08_Sub-controller_JSON_Command_Set#CMD_MODULE_TYPE
            # self.base_control_low.send_command({"T":300,"mode":0,"mac":"EF:EF:EF:EF:EF:EF"})  # the base won't be ctrl by esp-now broadcast cmd, but it can still recv broadcast megs.








            self.has_base_control_low_initialized = True

        except Exception as e:
            print(f"[CB] Error initializing low level control: {e}")
            self.has_base_control_low_initialized = False

    def send_command(self, command):
        self.base_control_low.send_command(command)

    # def move_forward(self, speed, duration):
    #     self.base_control_low.send_command({"T": 1, "L": speed, "R": speed})
    #     if duration > 0:
    #         time.sleep(duration) # FIXME: blocks the whole robot
    #         self.base_control_low.send_command({"T": 1, "L": 0, "R": 0})

    def set_turret_pan_tilt(self, pan, tilt):
        pass
        # self.base_control.send_command({"T": 900, "main": 3, "module": 2, "pan": pan, "tilt": tilt})

    # def set_speed(self, left_speed, right_speed):
    #     self.base_control.set_speed(left_speed, right_speed)

    # def stop(self):
    #     self.base_control.stop()

    # def cleanup(self):
    #     self.base_control.cleanup()