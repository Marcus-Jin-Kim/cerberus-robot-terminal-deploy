# cb ai turret udp + flask mjpeg server
import yaml
# from cb_low_level_control import CBLowLevelControl
from cb_robot_control import CBRobotControl
import socket, time, json, threading
from typing import Optional
from urllib.parse import parse_qs

# Flask for MJPEG stream
from flask import Flask, Response, jsonify

import traceback


class CerberusRobotTerminalServer:

    def __init__(self, config_file="cb_config.yaml",                 
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
        self._lock = threading.Lock()
        self.last_json: Optional[dict] = None     # JSON-safe pose data (no bytes)
        self.last_jpeg: Optional[bytes] = None    # latest JPEG bytes
        self.last_ts: float = 0.0

        # Flask app        
        from cb_cmd_routes import create_routes_blueprint
        self.app = Flask(__name__)
        self.app.register_blueprint(create_routes_blueprint(self))
        # self._setup_routes()

        print(f"[SERV] UDP on {self.host}:{self.udp_port}, HTTP MJPEG on {self.host}:{self.http_port}")


    def mjpeg_gen(self):
        period = 1.0 / max(1, self.stream_fps)
        while True:
            t0 = time.time()
            with self._lock:
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
            with self._lock:
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

    def start(self):
        # Threads: robot_control + UDP; Flask runs in main thread
        threading.Thread(target=self._robot_control_loop, daemon=True).start()
        threading.Thread(target=self._udp_loop, daemon=True).start()
        # Blocking HTTP server (no reloader so threads are preserved)
        self.app.run(host=self.host, port=self.http_port, threaded=True, use_reloader=False, debug=False)
    
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

if __name__ == "__main__":
    # One backend instance shared across UDP and Flask
    server = CerberusRobotTerminalServer(
        # host="0.0.0.0", udp_port=5001, http_port=5100,
        # return_image=True, control_turret=True, auto_aim=True
    )
    server.start()