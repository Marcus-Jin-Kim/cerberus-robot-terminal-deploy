import time
from flask import Flask, Response, render_template_string, jsonify
import threading

from WavesharePi.ugv_rpi.cb_ai_turret_backend import CerberusAITurretBackend

# Simple viewer page
INDEX_HTML = """
<!doctype html>
<html>
  <head><meta charset="utf-8"><title>Pose Stream</title></head>
  <body style="margin:0;background:#000;display:flex;justify-content:center;align-items:center;height:100vh">
    <img src="/stream" style="max-width:100%;height:auto;" />
  </body>
</html>
"""
aiTurret = CerberusAITurretBackend(
    return_image=True,control_turret=True,auto_aim= False) # True)
app = Flask(__name__)



@app.route("/")
def index():
    return render_template_string(INDEX_HTML)

# _last_lock = threading.Lock()

def frame_generator():
    
    period = 1.0 / max(1, aiTurret.turret_cam_fps_cap)
    while True:
        t0 = time.time()
        print("calling detect_nose")
        r = aiTurret.detect_nose()
        if r["OK"]:
            jpg = r["data"]["jpg_image_bytes"]
            # print(r["data"])
                              

            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            
            # yield (b"--frame\r\n"
            #    b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        else:
            time.sleep(0.01)
            # continue

        # FPS cap
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)

@app.route("/stream")
def stream():
    return Response(frame_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/nose")
def nose():
    r = aiTurret.detect_nose()
    return jsonify(r)
    # with _last_lock:
    #     data = dict(last_nose)
    # return jsonify(data)

if __name__ == "__main__":
    try:
        # Access from other machines: http://<pi-ip>:5000/
        app.run(host="0.0.0.0", port=5100, threaded=True)
    finally:
        aiTurret.release_cv2()
    #     cap.release()