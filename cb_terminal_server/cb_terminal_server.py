# cb ai turret udp + flask mjpeg server
import os, sys, signal, atexit
import yaml
# from cb_low_level_control import CBLowLevelControl
from cb_robot_control import CBRobotControl
import socket, time, json, threading
from typing import Optional
from urllib.parse import parse_qs

# Flask for MJPEG stream
from flask import Flask, Response, jsonify
from flask_socketio import SocketIO, emit
import traceback
from cb_cmd_routes import create_routes_blueprint

class CerberusRobotTerminalServer:

    def __init__(self, config_file="../cb_config.yaml",                 
        # , host='0.0.0.0', udp_port=5001, http_port=5100,
        #    return_image_override=None, control_turret_override=None, auto_aim_override=None
         ):

        self.config = self._load_config(config_file)

        self.host = self.config.get("ROBOT_TERMINAL_SERVER_HOST") #, "0.0.0.0")
        self.udp_port = self.config.get("ROBOT_TERMINAL_SERVER_UDP_PORT") #, 5001)
        self.http_port = self.config.get("ROBOT_TERMINAL_SERVER_HTTP_PORT") # , 5100)
        self.control_hz = self.config.get("ROBOT_TERMINAL_SERVER_ROBOT_CONTROL_HZ") # , 60)
        self.stream_fps = self.config.get("ROBOT_TERMINAL_SERVER_STREAM_FPS") # , 15)

        print(f"[SERV] Robot control HZ = {self.control_hz}")

        # UDP socket
        self.udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_server.bind((self.host, self.udp_port))

        # Shared backend (single instance)
        # self.low_level_control = CBLowLevelControl(self.config)
        self.robot_control = CBRobotControl(
            config=self.config,
            # cb_low_level_control=self.low_level_control            
        )

        # Shared state
        # self._lock = threading.Lock() # lets remove lock for now for testing
        self.last_json: Optional[dict] = None     # JSON-safe pose data (no bytes)
        self.last_jpeg: Optional[bytes] = None    # latest JPEG bytes
        self.last_ts: float = 0.0


        # self._setup_routes()

        print(f"[SERV] UDP on {self.host}:{self.udp_port}, HTTP MJPEG on {self.host}:{self.http_port}")


    def mjpeg_gen(self):
        period = 1.0 / max(1, self.stream_fps)
        while True:
            t0 = time.time()
            # with self._lock:
            jpg = self.last_jpeg
            if jpg:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    def _robot_control_loop(self):
        # Capture/process loop: updates shared JSON and JPEG once per cycle
        while True:
            t0 = time.time()
            wait_for_second = 1.0 / max(1, self.control_hz)
            try:
                res = self.robot_control.scan_enemy()
            except Exception as e:
                tb = traceback.extract_tb(e.__traceback__)[-1]
                print(f"[AI ] detect_pose error at {tb.filename}:{tb.lineno} in {tb.name}: {e}")
                time.sleep(0.05)
                continue

           # Build outputs OUTSIDE the lock
            jpeg: bytes = None
            if isinstance(res, dict) and res.get("return_image", False):
                v = (res.get("data") or {}).get("jpg_image_bytes")
                jpeg = v
                # if isinstance(v, bytes):
                #     jpeg = v                    # zero-copy
                # elif isinstance(v, (bytearray, memoryview)):
                #     jpeg = bytes(v)             # one copy to immutable

            # copy res to self.last_json except ["data"]["jpg_image_bytes"]
            clean = {"OK": False, "error": "bad_result"}
            if isinstance(res, dict):
                clean = res.copy()              # shallow copy
                d = clean.get("data")
                if isinstance(d, dict):
                    d = d.copy()
                    d.pop("jpg_image_bytes", None)
                    clean["data"] = d

            # Minimal critical section
            #with self._lock: # disable lock for now
            self.last_jpeg = jpeg           # may be None if not returning image
            self.last_json = clean
            self.last_ts = time.time()

            dt = time.time() - t0
            if dt < wait_for_second:
                time.sleep(wait_for_second - dt)

    def _udp_loop(self):
        # Serve small JSON (no images) over UDP
        self.udp_server.settimeout(0.5)  # set once
        while True:
            try:
                data, addr = self.udp_server.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[UDP] recv error: {e}")
                continue

            # Expect URL-encoded query, e.g., b"req=mp-pose"
            try:
                text = data.decode("utf-8", errors="ignore").strip()
                if not text:
                    continue
                qs = parse_qs(text, keep_blank_values=True)
                req = (qs.get("req") or [""])[0]
            except Exception:
                continue

            if req == "mp-pose":
                with self._lock:
                    reply = self.last_json if self.last_json is not None else {"OK": False, "error": "no_data"}
                try:
                    # just drop jpeg_image_bytes in data since udp sever doesnt serve it
                    if "data" in reply and isinstance(reply["data"], dict):
                        reply["data"].pop("jpeg_image_bytes", None)

                    payload = json.dumps(reply).encode("utf-8")                    
                    self.udp_server.sendto(payload, addr)

                except Exception as e:
                    print(f"[UDP] send error: {e}")

    # def start_old(self, run_robot_control_loop=True, run_udp_loop=True):
    #     # Threads: robot_control + UDP; Flask runs in main thread
    #     if run_robot_control_loop:
    #         threading.Thread(target=self._robot_control_loop, daemon=True).start()
    #     if run_udp_loop:
    #         threading.Thread(target=self._udp_loop, daemon=True).start()
    #     # Blocking HTTP server (no reloader so threads are preserved)
    #     self.app.run(host=self.host, port=self.http_port, threaded=True, use_reloader=False, debug=False)

    # def start(self):
    #     app.run(host=self.host, port=self.http_port, threaded=True, use_reloader=False, debug=False)
        # this is main loop from app. it doesn use flask
        #     if __name__ == "__main__":
        # # lights off
        # base.lights_ctrl(255, 255)
        
        # # play a audio file in /sounds/robot_started/
        # audio_ctrl.play_random_audio("robot_started", False)

        # # update the size of videos and pictures
        # si.update_folder(thisPath)

        # # pt/arm looks forward
        # if f['base_config']['module_type'] == 1:
        #     base.base_json_ctrl({"T":f['cmd_config']['cmd_arm_ctrl_ui'],"E":f['args_config']['arm_default_e'],"Z":f['args_config']['arm_default_z'],"R":f['args_config']['arm_default_r']})
        # else:
        #     base.gimbal_ctrl(0, 0, 200, 10)

        # # feedback loop starts
        # si.start()
        # si.resume()
        # data_update_thread = threading.Thread(target=update_data_loop, daemon=True)
        # data_update_thread.start()

        # # base data update
        # base_update_thread = threading.Thread(target=base_data_loop, daemon=True)
        # base_update_thread.start()

        # # lights off
        # base.lights_ctrl(0, 0)
        # cmd_on_boot()

        # # run the main web app
        # socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)


        # following app.py
        # base.lights_ctrl(255, 255)
        
        # # play a audio file in /sounds/robot_started/
        # audio_ctrl.play_random_audio("robot_started", False)

        # # update the size of videos and pictures
        # si.update_folder(thisPath)

        # # pt/arm looks forward
        # if f['base_config']['module_type'] == 1:
        #     base.base_json_ctrl({"T":f['cmd_config']['cmd_arm_ctrl_ui'],"E":f['args_config']['arm_default_e'],"Z":f['args_config']['arm_default_z'],"R":f['args_config']['arm_default_r']})
        # else:
        #     base.gimbal_ctrl(0, 0, 200, 10)

        # # feedback loop starts
        # si.start()
        # si.resume()
        # data_update_thread = threading.Thread(target=update_data_loop, daemon=True)
        # data_update_thread.start()

        # # base data update
        # base_update_thread = threading.Thread(target=base_data_loop, daemon=True)
        # base_update_thread.start()

        # # lights off
        # base.lights_ctrl(0, 0)
        # cmd_on_boot()

        # # run the main web app
        # socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)


        
    def _load_config(self, config_file):
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                print(f"[SERV] Loaded config: {config}")
                return config
        except Exception as e:
            print(f"[SERV] Error loading config: {e}")
            return {}
        pass



app = Flask(__name__)
socketio = SocketIO(app)

if __name__ == "__main__":
    # write down pid
    pid_file_path = "../cb_pid_terminal.txt"
    with open(pid_file_path, "w") as f:
        f.write(str(os.getpid()) + "\n")

    def _cleanup_pidfile():
        if os.path.exists(pid_file_path):
            os.remove(pid_file_path)
            print("[SERV] PID file removed.")

    def _on_signal(signum, frame):
        print(f"[SERV] signal {signum} received; exiting")
        _cleanup_pidfile()
        sys.exit(0)

    atexit.register(_cleanup_pidfile)
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    # def cmd_on_boot(server: CerberusRobotTerminalServer):
    #     with open('/home/ws/ugv_rpi' + '/config.yaml', 'r') as yaml_file:
    #         f = yaml.safe_load(yaml_file)
    #     cmd_list = [
    #         {"T":142,"cmd":50},   # set feedback interval
    #         {"T":131,"cmd":1},    # serial feedback flow on:
    #         #################
    #         ## !!!! feedback flow MUST BE ON TO GET FEEDBACK DATA AND UPDATE ROS POSITION TF AND EVERYTHING
    #         ##################
    #         #             
    #         {"T":143,"cmd":0},    # serial echo off ## MAYBE THIS IS IMPORTANT?
    #         {"T":4,"cmd":f['base_config']['module_type']}, # select the module - 0:None 1:RoArm-M2-S 2:Gimbal
    #         {"T":300,"mode":0,"mac":"EF:EF:EF:EF:EF:EF"},  # the base won't be ctrl by esp-now broadcast cmd, but it can still recv broadcast megs.

    #         # 'send -a -b'    # add broadcast mac addr to peer
    #         # 'base -c {"T":142,"cmd":50}',   # set feedback interval
    #         # 'base -c {"T":131,"cmd":1}',    # serial feedback flow on
    #         # 'base -c {"T":143,"cmd":0}',    # serial echo off
    #         # 'base -c {{"T":4,"cmd":{}}}'.format(f['base_config']['module_type']),      # select the module - 0:None 1:RoArm-M2-S 2:Gimbal
    #         # 'base -c {"T":300,"mode":0,"mac":"EF:EF:EF:EF:EF:EF"}',  # the base won't be ctrl by esp-now broadcast cmd, but it can still recv broadcast megs.
    #         # 'send -a -b'    # add broadcast mac addr to peer
    #     ]
    #     print('{{"T":4,"cmd":{}}}'.format(f['base_config']['module_type']))
    #     for i in range(0, len(cmd_list)):
    #         server.robot_control.low_level_control.base_control_low.send_command(cmd_list[i])
    #         # cvf.info_update(cmd_list[i], (0,255,255), 0.36)
    #     # set_version(f['base_config']['main_type'], f['base_config']['module_type'])

    # One backend instance shared across UDP and Flask
    server = CerberusRobotTerminalServer(
        # host="0.0.0.0", udp_port=5001, http_port=5100,
        # return_image=True, control_turret=True, auto_aim=True
    )
    app.register_blueprint(create_routes_blueprint(server))

    try:
        # Threads: robot_control + UDP; Flask runs in main thread
        threading.Thread(target=server._robot_control_loop, daemon=True).start()
        threading.Thread(target=server._udp_loop, daemon=True).start()

        # app.run(host=server.host, port=server.http_port, threaded=True, use_reloader=False, debug=False)
        server.robot_control.low_level_control.base_control_low.lights_ctrl(0, 0)
        # cmd_on_boot(server)
        socketio.run(app, host='0.0.0.0', port=server.http_port, allow_unsafe_werkzeug=True)
    finally:
        _cleanup_pidfile()
        print("[SERV] Server stopped.")
