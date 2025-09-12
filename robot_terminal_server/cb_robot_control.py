import threading
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

    """
    Singleton top-level robot controller.
    Use CBRobotControl(config, ...) normally; only first call initializes.
    Access existing instance with CBRobotControl.instance().
    """

    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        # Lazy singleton allocation
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, config, # cb_low_level_control:CBLowLevelControl, 
                 # return_image=False, control_turret=False, auto_aim=False : depricated. using config
                 ):
        # Prevent re-running initialization
        if getattr(self, "_initialized", False):
            print(f"[CB] RobotControl Singleton already initialized; ignoring new init.")
            # # Optional: warn if parameters changed
            # if (config is not self.config) or \
            #    (return_image != self.turret.return_image or
            #     control_turret != self.turret.control_turret or
            #     auto_aim != self.turret.auto_aim):
            #     print("[CBRobotControl] Singleton already initialized; ignoring new params.")
            return
        
        print(f"[CB] Initializing low level control ...")
        self.config = config
        self.low_level_control = CBLowLevelControl(self.config)


        # if cb_low_level_control is None:
        #     # raise value error and print the current file name
        #     raise ValueError(f"[{Path(__file__).name}] cb_low_level_control cannot be None")

        print(f"[CB] Initializing body control ...")
        self.body_control = CBBodyControl(
            self.config,
            self.low_level_control
        )

        print(f"[CB] Initializing turret control ...")
        self.turret = CBTurretControl(
            self.config,
            self.low_level_control, 
            # depricated # return_image=return_image,             # control_turret=control_turret,             # auto_aim=auto_aim
        )
        # print(f"[CB]Turret Control init: control_turret={self.turret.control_turret}, auto_aim={self.turret.auto_aim}, return_image={self.turret.return_image}")

        self._initialized = True  # mark done

        print(f"[CB] CBRobotControl singleton initialized.")


    @classmethod
    def instance(cls):
        """Return existing singleton or None if not created yet."""
        return cls._instance

    @classmethod
    def reset_for_tests(cls):
        """Testing helper: drop singleton (not for production)."""
        with cls._lock:
            cls._instance = None


    def scan_enemy(self, auto_aim=False):
        return self.turret.detect_pose()



    