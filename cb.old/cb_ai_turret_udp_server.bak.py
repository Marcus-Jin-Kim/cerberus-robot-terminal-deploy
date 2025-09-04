# cb ai turret udp server

from cb_ai_turret_backend import CerberusAITurretBackend
import socket
import urllib.parse
import time
import json



class CerberusAITurretUDPServer:
    def __init__(self, host='0.0.0.0', port=5001, return_image=False, control_turret=False, auto_aim=False):
        self.host = host
        self.port = port
        self.udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_server.bind((self.host, self.port))
        self.ai_turret = CerberusAITurretBackend(return_image=return_image, control_turret=control_turret, auto_aim=auto_aim)
        self.last_data = None
        self.last_timestamp = 0.0
        print(f"[UDP] Turret UDP server started on {self.host}:{self.port}")

    def start(self):
        # run backend in a loop
        while True:
            wait_for_second = 1.0 / max(1, self.ai_turret.turret_cam_fps_cap)
            t0 = time.time()
            r = self.ai_turret.detect_nose()
            
            self.last_data = r
            # debug print            
            # print(f"[AI] detect_nose result: {self.last_data}")
            print(json.dumps(self.last_data))


            self.last_timestamp = time.time()

            # listen for UDP requests
            # non-blocking with timeout, timeout max to fps cap
            self.udp_server.settimeout(wait_for_second)
            try:
                data, addr = self.udp_server.recvfrom(1024)
                print(f"[UDP] Received data from {addr}: {data}")
                self.handle_request(data, addr)
            except socket.timeout:
                pass
            except Exception as e:
                print(f"[UDP] Error receiving data: {e}")
            
            # FPS cap
            dt = time.time() - t0
            if dt < wait_for_second:
                time.sleep(wait_for_second - dt)
                continue

        
        # while True:
        #     data, addr = self.udp_server.recvfrom(1024)
        #     print(f"[UDP] Received data from {addr}: {data}")
        #     self.handle_request(data, addr)

    def handle_request(self, data, addr):
        try:
            message = json.loads(data)
            if message.get("action") == "detect_nose":
                print("[UDP] Handling detect_nose request")
                pass
                # result = self.ai_turret.detect_nose()
                # self.udp_server.sendto(json.dumps(result).encode(), addr)
                # self.udp_server.sendto(json.dumps(self.last_data).encode(), addr)
        except Exception as e:
            print(f"[UDP] Error handling request: {e}")

if __name__ == "__main__":
    server = CerberusAITurretUDPServer(return_image=True, control_turret=True, auto_aim=True)
    server.start()