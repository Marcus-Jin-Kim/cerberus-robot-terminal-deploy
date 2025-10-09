import time
import cv2
# from flask import Flask, Response, render_template_string, jsonify
import mediapipe as mp
import threading

from cb_low_level_control import CBLowLevelControl

from pathlib import Path
import traceback

#TODO: ugv_rpi parameters. set it to yaml or something
# TURRET_CAM_DEVICE_INDEX = 0
# TURRET_CAM_WIDTH = 640
# TURRET_CAM_HEIGHT = 480
# TURRET_CAM_FPS_CAP = 30
# TURRET_CAM_JPEG_QUALITY = 80
# TURRET_CAM_FOV = 65
# TURRET_HOR_MIN = -135 # UGV max is 180
# TURRET_HOR_MAX = 135 # UGV max is -180
# TURRET_VER_MIN = -25 # UGV max is -30
# TURRET_VER_MAX = 45 # UGV max is 90
# TURRET_HOR_CENTER = 0
# TURRET_VER_CENTER = 0
# TURRET_HOR_STEP_MAX = 20 # 10
# TURRET_VER_STEP_MAX = 20 # 10
# TURRET_SPEED = 0 # 300 UGV visual control /0 = max speed
# TURRET_ACC = 0 # 500 # 300 # 0 = max acc
# TURRET_AIM_TOLERANCE_PX = 50  # pixels
# TURRET_AIM_SMOOTHING = 0.25  # 0.5 too much, 0.3 too much     # 0.0 to 1.0, higher is smoother but more laggy
# TURRET_AIM_CMD_DELAY = 0.05
# command list
# WavesharePi/ugv_rpi/tutorial_en/08 Microcontroller JSON Command Set.ipynb


class CBTurretControl: # this should be top level robot controller later

    def __init__(self, config, cb_low_level_control:CBLowLevelControl, 
                #  turret_cam_device_index=0, 
                #  turret_cam_width=640, turret_cam_height=480, turret_cam_fov= TURRET_CAM_FOV,
                #  turret_cam_fps_cap=30, turret_cam_jpeg_quality=80, 
                #  return_image=False, control_turret=False, auto_aim=False
                 ):
        
        self.config = config

        if cb_low_level_control is None:
            # raise value error and print the current file name
            raise ValueError(f"[{Path(__file__).name}] cb_low_level_control cannot be None")
        # self.body_control = CBBodyControl(cb_low_level_control)
        self.turret_low_level_control = cb_low_level_control

        self.turret_cam_device_index = self.config["TURRET_CAM_DEVICE_INDEX"]
        self.turret_cam_width = self.config["TURRET_CAM_WIDTH"]
        self.turret_cam_height = self.config["TURRET_CAM_HEIGHT"]
        self.turret_cam_fps_cap = self.config["TURRET_CAM_FPS_CAP"]
        self.turret_cam_jpeg_quality = self.config["TURRET_CAM_JPEG_QUALITY"]
        self.turret_cam_fov = self.config["TURRET_CAM_FOV"]
        self.return_image = self.config["TURRET_CAM_RETURN_IMAGE"]

        print(f"[CB]Turret camera: {self.turret_cam_fps_cap}FPS, JPEG quality={self.turret_cam_jpeg_quality}, return_image={self.return_image}")

        # Init camera
        self.cv2 = cv2.VideoCapture(self.turret_cam_device_index)

        self.cv2.set(cv2.CAP_PROP_FRAME_WIDTH, self.turret_cam_width)
        self.cv2.set(cv2.CAP_PROP_FRAME_HEIGHT, self.turret_cam_height)

        # Init MediaPipe once (runs native code under the hood)
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False,
                            model_complexity=0,            # 0 is faster on low-power CPUs
                            smooth_landmarks=True,
                            min_detection_confidence=0.5,
                            min_tracking_confidence=0.5)

        # self.last_nose = {"nose_detected": False, "x_norm": None, "y_norm": None, "z_norm": None,
        #                   "x_px": None, "y_px": None, "z_px": None,
        #                   "visibility": None, "ts": 0.0}
        self.last_pose = {"pose_detected": False, "landmarks": [], "ts": 0.0}
        self._last_lock = threading.Lock()



        # init turret control if enabled
        self.control_turret = self.config["TURRET_CONTROL_ENABLED"]
        self.auto_aim = self.config["TURRET_AUTO_AIM_ENABLED"]
        print(f"[CB]Turret control: {self.control_turret}, auto-aim: {self.auto_aim}")

        self.turret_last_pan = 0
        self.turret_last_tilt = 0
        self.turret_hor_min =  self.config["TURRET_HOR_MIN"]
        self.turret_hor_max =  self.config["TURRET_HOR_MAX"]
        self.turret_ver_min =  self.config["TURRET_VER_MIN"]
        self.turret_ver_max =  self.config["TURRET_VER_MAX"]
        self.turret_hor_center =  self.config["TURRET_HOR_CENTER"]
        self.turret_ver_center =  self.config["TURRET_VER_CENTER"]
        self.turret_step_hor_max =  self.config["TURRET_HOR_STEP_MAX"]
        self.turret_step_ver_max =  self.config["TURRET_VER_STEP_MAX"]
        self.turret_speed =  self.config["TURRET_SPEED"]
        self.turret_acc =  self.config["TURRET_ACC"]
        self.turret_aim_tolerance_px =  self.config["TURRET_AIM_TOLERANCE_PX"]
        self.turret_aim_smoothing =  self.config["TURRET_AIM_SMOOTHING"]
        self.turret_aim_cmd_delay =  self.config["TURRET_AIM_CMD_DELAY"]
        self.turret_is_aiming = False

        if self.control_turret:                        
            
            try:
                # using singleton base control
                # self.turret_control_base = BaseController('/dev/ttyAMA0', 115200)
                # self.turret_control_base.send_command({"T": 900, "main": 3, "module": 2})

                # FIXME: use robot_control's low level control 
                self.turret_low_level_control.base_control_low.gimbal_ctrl(0, 0, self.turret_speed, self.turret_acc) 
                self.turret_low_level_control.base_control_low.lights_ctrl(1, 1)
                
                self.turret_last_pan = 0
                self.turret_last_tilt = 0

                
                print(f"[CB]Turret control base initialized.")

                self.turret_is_aiming = False
                self.turret_aim_cmd_delay_start = 0
                self.turret_aim_cmd_delay_timer = 0

                
            except Exception as e:
                print(f"[CB]ERROR: Failed to init turret control base: {e}")
                # self.control_turret = False


        time.sleep(1) # wait for gimbal to init
        self.turret_low_level_control.base_control_low.lights_ctrl(0,0)
        print(f"[CB]AITurretBackend init: control_turret={self.control_turret}, auto_aim={self.auto_aim}, return_image={self.return_image}")                # self.auto_aim = False

    def detect_pose(self):
        ok, frame_bgr = self.cv2.read()
        if not ok:
            return {
                "OK": False,
                "e_msg": "ERROR_CV_READ_FROM_CAMERA_FAIL",
            }

        h, w = frame_bgr.shape[:2]

        # Downscale for pose to reduce CPU
        small_w, small_h = 320, 240 # move to config later
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frame_small = cv2.resize(frame_rgb, (small_w, small_h), interpolation=cv2.INTER_AREA)

        results = self.pose.process(frame_small)

        # Draw landmarks and extract NOSE (index 0)
        if results.pose_landmarks:
            self.mp_draw.draw_landmarks(frame_bgr, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
            # get pose landmarks and store to self.last_pose
            # do i need lock here?
            with self._last_lock:
                self.last_pose["pose_detected"] = True
                self.last_pose["frame_w"] = w
                self.last_pose["frame_h"] = h
                self.last_pose["landmarks"] = []
                for lm in results.pose_landmarks.landmark:
                    self.last_pose["landmarks"].append({
                        "x": round(float(lm.x),6),
                        "y": round(float(lm.y),6),
                        "z": round(float(lm.z),6),
                        "visibility": round(float(lm.visibility),6)
                    })
            self.last_pose["ts"] = time.time()

            # extract NOSE : FIXME: performance?
            nose_lm = self.last_pose["landmarks"][0]
            # x_norm, y_norm, z_norm = float(nose_lm["x"]), float(nose_lm["y"]), float(nose_lm["z"])
            x_px, y_px = int(nose_lm["x"] * w), int(nose_lm["y"] * h)
            z_px = nose_lm["z"] * w

            # draw a marker
            if self.return_image:
                
                cv2.circle(frame_bgr, (x_px, y_px), 6, (0, 255, 0), -1)
                # cv2.putText(frame_bgr, f"NOSE {x_px},{y_px},{z_norm}", (x_px + 8, y_px - 8,z_norm),
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                cv2.putText(frame_bgr, f"NOSE {x_px},{y_px}", (x_px + 8, y_px - 8),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)



            if self.turret_is_aiming:
                self.turret_aim_cmd_delay_timer += time.time() - self.turret_aim_cmd_delay_start
                if self.turret_aim_cmd_delay_timer >= self.turret_aim_cmd_delay:
                    self.turret_is_aiming = False
                    self.turret_aim_cmd_delay_timer = 0
            

            if self.control_turret and self.auto_aim and self.turret_is_aiming == False:

                # since i know fov and image width/height, i can calculate the the angle difference
                err_x = x_px - (w // 2)
                err_y = y_px - (h // 2)
                angle_per_pixel = self.turret_cam_fov / w
                angle_err_x = err_x * angle_per_pixel * self.turret_aim_smoothing
                if angle_err_x > self.turret_step_hor_max:
                    angle_err_x = self.turret_step_hor_max
                elif angle_err_x < -self.turret_step_hor_max:
                    angle_err_x = -self.turret_step_hor_max

                angle_err_y = err_y * angle_per_pixel * self.turret_aim_smoothing
                if angle_err_y > self.turret_step_ver_max:
                    angle_err_y = self.turret_step_ver_max
                elif angle_err_y < -self.turret_step_ver_max:
                    angle_err_y = -self.turret_step_ver_max

                # print(f"err_x: {err_x}, err_y: {err_y}, angle_err_x: {angle_err_x}, angle_err_y: {angle_err_y}")                

                new_pan = self.turret_last_pan + int(angle_err_x)
                new_tilt = self.turret_last_tilt - int(angle_err_y)
                # limit pan/tilt
                if new_pan > self.turret_hor_max:
                    new_pan = self.turret_hor_max
                elif new_pan < self.turret_hor_min:
                    new_pan = self.turret_hor_min
                if new_tilt > self.turret_ver_max:
                    new_tilt = self.turret_ver_max
                elif new_tilt < self.turret_ver_min:
                    new_tilt = self.turret_ver_min

                # print(f"Turret move to pan: {new_pan}, {self.turret_last_pan} tilt: {new_tilt}, {self.turret_last_tilt}")
                # FIXME: TOO LOW
                self.turret_low_level_control.base_control_low.gimbal_ctrl(new_pan, new_tilt, self.turret_speed, self.turret_acc)
                self.turret_last_pan = new_pan
                self.turret_last_tilt = new_tilt
                self.turret_is_aiming = True
                self.turret_aim_cmd_delay_start = time.time()
                self.turret_aim_cmd_delay_timer = 0


        else:
            with self._last_lock:
                self.last_pose["pose_detected"] = False
                # self.last_nose["norm"] = None
                # self.last_nose["px"] = None
                # self.last_nose["z"] = None
                # self.last_nose["visibility"] = None
                # self.last_nose["ts"] = time.time()

                


        jpg_image_bytes = None
        if self.return_image:
            # Encode to JPEG and yield multipart chunk
            ok, jpg = cv2.imencode('.jpg', frame_bgr, 
                            [int(cv2.IMWRITE_JPEG_QUALITY), self.turret_cam_jpeg_quality])
            if not ok:
                return {
                    "OK": False,
                    "e_msg": "ERROR_CV_IMENCODE_FAIL",
                }
            jpg_image_bytes = jpg.tobytes()

        data = self.last_pose
        data["pose_detected"] = self.last_pose["pose_detected"]
        data["jpg_image_bytes"] = jpg_image_bytes


        return {
            "OK": True,
            "e_msg": "OK",
            "pan": self.turret_last_pan,
            "tilt": self.turret_last_tilt,
            "return_image": self.return_image,
            "data": data
        }

        # yield (b"--frame\r\n"
        #         b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")

    def release_cv2(self):
        self.cv2.release()

    # on destroy release cv2
    def __del__(self):
        self.release_cv2()



# test turret control
if __name__ == "__main__":# T
    import yaml, os, sys
    # os.chdir("/home/ws/cb")
    # # sys.path.append("../")
    # sys.path.append("/home/ws/ugv_rpi")  # for cb_low_level_control
    
    with open("cb_config.yaml", "r") as f:
        config = yaml.safe_load(f)
    
    # override config
    config["TURRET_CONTROL_ENABLED"] = True
    config["TURRET_AUTO_AIM_ENABLED"] = True
    config["TURRET_CAM_RETURN_IMAGE"] = False

    # get CBLowLevelControl 
    cb_low_level_control = CBLowLevelControl(config)

    aiTurret = CBTurretControl(config, cb_low_level_control)
    try:
        while True:
            wait_for_second = 1.0 / max(1, aiTurret.turret_cam_fps_cap)
            t0 = time.time()
            r = aiTurret.detect_pose()
            # print(r["e_msg"])
            # if r["OK"]:
            #     if r["pose_detected"]:
            #         print(f"[POSE] {len(r['data']['landmarks'])} landmarks detected.")
            #         # print(r["data"])
            #     else:
            #         print("[POSE] no pose detected.")
            # else:
            #     print(f"[POSE] ERROR: {r['e_msg']}")
            #     continue

            dt = time.time() - t0
            if dt < wait_for_second:
                time.sleep(wait_for_second - dt)
                continue
    finally:
        aiTurret.release_cv2()
        print("aiTurret CV2 released.")


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


    