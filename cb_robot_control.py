import time
import cv2
# from flask import Flask, Response, render_template_string, jsonify
# import mediapipe as mp
# import threading

from cb_low_level_control import CBLowLevelControl
from cb_body_control import CBBodyControl
from cb_turret_control import CBTurretControl

from pathlib import Path
import yaml
import traceback


class CBRobotControl: # this should be top level robot controller later

    def __init__(self, cb_low_level_control:CBLowLevelControl, 
                 return_image=False, control_turret=False, auto_aim=False):
        
        self.config = self._read_config()

        if cb_low_level_control is None:
            # raise value error and print the current file name
            raise ValueError(f"[{Path(__file__).name}] cb_low_level_control cannot be None")

        self.body_control = CBBodyControl(cb_low_level_control)
        self.turret = CBTurretControl(cb_low_level_control,return_image=return_image, control_turret=control_turret, auto_aim=auto_aim)


        print(f"[CB]Turret Control init: control_turret={self.turret.control_turret}, auto_aim={self.turret.auto_aim}, return_image={self.turret.return_image}")


    def scan_enemy(self, auto_aim=False):
        return self.turret.detect_pose()

    def _read_config(self):
        try:
            with open("cb_config.yaml", 'r') as f:
                config = yaml.safe_load(f)
                print(f"[CB] Loaded config: {config}")
                return config
        except Exception as e:
            print(f"[CB] Error loading config: {e}")
            return {}
        pass

# if __name__ == "__main__":
#     aiTurret = CBRobotControl()
#     try:
#         while True:
#             wait_for_second = 1.0 / max(1, aiTurret.turret_cam_fps_cap)
#             t0 = time.time()
#             r = aiTurret.detect_nose()
#             if r["OK"]:
#                 print(r["data"])
            
#             dt = time.time() - t0
#             if dt < wait_for_second:
#                 time.sleep(wait_for_second - dt)
#                 continue

#     finally:
#         aiTurret.release_cv2()
#         print("aiTurret CV2 released.")


    