# cb ai turret udp + flask mjpeg server
import os, sys, signal, atexit
import yaml
# from cb_low_level_control import CBLowLevelControl
from cb_robot_control import CBRobotControl
import socket, time, json, threading
from typing import Optional
import urllib
from urllib.parse import parse_qs

# Flask for MJPEG stream
from flask import Flask, Response, jsonify
from flask_socketio import SocketIO, emit
import traceback
# from cb_cmd_routes import create_routes_blueprint
from cb_routes_cmd import create_routes_blueprint_cmd
from cb_routes_dev import create_routes_blueprint_dev

class CerberusRobotTerminalServer:

    def __init__(self, config_file="../cb_config.yaml",                 
        # , host='0.0.0.0', udp_port=5001, http_port=5100,
        #    return_image_override=None, control_turret_override=None, auto_aim_override=None
         ):

        self.config = self._load_config(config_file)

        self.host = self.config.get("ROBOT_TERMINAL_SERVER_HOST") #, "0.0.0.0")
        self.udp_port = self.config.get("ROBOT_TERMINAL_SERVER_UDP_PORT") #, 5001)
        self.http_port = self.config.get("ROBOT_TERMINAL_SERVER_HTTP_PORT") # , 5100)
        
        # self.control_hz = self.config.get("ROBOT_TERMINAL_SERVER_ROBOT_CONTROL_HZ") # , 60)
        self.udp_server_enabled = self.config.get("ROBOT_UDP_SERVER_ENABLED", False)
        self.scan_enemy_enabled = self.config.get("ROBOT_SCAN_ENEMY_ENABLED", False)    
        self.scan_enemy_hz = self.config.get("ROBOT_TERMINAL_SERVER_SCAN_ENEMY_HZ") # , 10)
        self.stream_fps = self.config.get("ROBOT_TERMINAL_SERVER_STREAM_FPS") # , 15)
        self.status_report_interval = self.config.get("ROBOT_STATUS_REPORT_TO_SKYNET_INTERVAL", 10)

        if self.udp_server_enabled is True:
            # UDP socket
            self.udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_server.bind((self.host, self.udp_port))
            print(f"[RTerm] UDP server on {self.host}:{self.udp_port}")

        else:            
            print(f"[RTerm] UDP server disabled.")

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
        self.last_report_since: float = 0.0        

        # self._setup_routes()
        print(f"[RTerm] scan_enemy_enabled = {self.scan_enemy_enabled}, scan enemy HZ = {self.scan_enemy_hz}")
        print(f"[RTerm] HTTP Server: {self.host}:{self.http_port}")


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
        # FIXME: better cycle management needed. robot can scan_enemy or can do other things
        while True:
            t0 = time.time()
            
            # report my robot status to skynet server first
            if self.last_report_since + self.status_report_interval < time.time():
                self.report_my_robot_status_to_skynet()
                self.last_report_since = time.time()
                # print(f"[SERV] report_my_robot_status_to_skynet")    

            # scan enemy part (make a function later)
            if self.scan_enemy_enabled:                
            
                wait_for_second = 1.0 / max(1, self.scan_enemy_hz)
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
                reply = self.last_json if self.last_json is not None else {"OK": False, "error": "no_data"}
                try:
                    # just drop jpeg_image_bytes in data since udp sever doesnt serve it
                    if "data" in reply and isinstance(reply["data"], dict):
                        reply["data"].pop("jpeg_image_bytes", None)

                    payload = json.dumps(reply).encode("utf-8")                    
                    self.udp_server.sendto(payload, addr)

                except Exception as e:
                    print(f"[UDP] send error: {e}")


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

    def report_my_robot_status_to_skynet(self):
        url_root = self.config.get("SKYNET_SERVER_URL_ROOT")
        machine_id = self.config.get("ROBOT_MACHINE_ID")
        try:
            resp = urllib.request.urlopen(f"{url_root}/report-my-robot-status-mid/{machine_id}", timeout=1.0)
            if resp.status_code == 200:
                # print(f"[SERV] report_my_robot_status_to_skynet OK")
                pass
            else:
                print(f"[SERV] report_my_robot_status_to_skynet error: {resp.status_code}")
        except Exception as e:
            print(f"[SERV] report_my_robot_status_to_skynet exception: {e}")
    

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
    app.register_blueprint(create_routes_blueprint_dev(server))
    app.register_blueprint(create_routes_blueprint_cmd(server))
    

    try:
        # Threads: robot_control + UDP; Flask runs in main thread
        threading.Thread(target=server._robot_control_loop, daemon=True).start()
        if server.udp_server_enabled is True:
            threading.Thread(target=server._udp_loop, daemon=True).start()

        # app.run(host=server.host, port=server.http_port, threaded=True, use_reloader=False, debug=False)
        server.robot_control.low_level_control.base_control_low.lights_ctrl(0, 0)
        # cmd_on_boot(server)
        socketio.run(app, host='0.0.0.0', port=server.http_port, allow_unsafe_werkzeug=True)
    finally:
        _cleanup_pidfile()
        print("[SERV] Server stopped.")
