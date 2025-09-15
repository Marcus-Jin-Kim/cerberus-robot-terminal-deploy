import os
from cb_terminal_server import CerberusRobotTerminalServer
from flask import Blueprint, Response, jsonify, request
import time

def create_routes_blueprint(robot_control_server:CerberusRobotTerminalServer):

    bp = Blueprint("cmd_routes", __name__)
    _bc = robot_control_server.robot_control.body_control

    @bp.route("/stream")
    def stream():
        return Response(robot_control_server.mjpeg_gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @bp.route("/get-mp-pose")
    def get_mp_pose():
        with robot_control_server._lock:
            data = robot_control_server.last_json if robot_control_server.last_json is not None else {"OK": False}
        return jsonify(data)

    @bp.route('/forward', methods = ['GET', 'POST'])
    def forward():    
        speed = request.args.get('speed', default=0, type=float)
        duration = request.args.get('duration', default=0, type=float)
        r = _bc.forward(speed=speed, duration= duration)    
        return jsonify(r),200

        
    @bp.route('/back', methods = ['GET', 'POST'])
    def backward():
        speed = request.args.get('speed', default=0, type=float)    
        duration = request.args.get('duration', default=0, type=float)
        r = _bc.back(speed=speed, duration=duration)
        return jsonify(r),200

    @bp.route('/stop', methods = ['GET', 'POST'])
    def stop():
        r = _bc.stop()
        return jsonify(r),200

        
    @bp.route('/moveleft', methods = ['GET', 'POST'])
    def moveleft():
        speed = request.args.get('speed', default=0, type=float)
        duration = request.args.get('duration', default=0, type=float)
        r = _bc.moveleft(speed=speed, duration=duration)
        return jsonify(r),200

    @bp.route('/moveright', methods = ['GET', 'POST'])
    def moveright():
        speed = request.args.get('speed', default=0, type=float)
        duration = request.args.get('duration', default=0, type=float)
        r = _bc.moveright(speed=speed, duration=duration)
        return jsonify(r),200

        
    @bp.route('/turnleft', methods = ['GET', 'POST'])
    def turnleft():
        speed = request.args.get('speed', default=0, type=float)
        duration = request.args.get('duration', default=0, type=float)
        r = _bc.turnleft(speed=speed, duration=duration)
        return jsonify(r),200

        
    @bp.route('/turnright', methods = ['GET', 'POST'])
    def turnright():
        speed = request.args.get('speed', default=0, type=float)
        duration = request.args.get('duration', default=0, type=float)
        r = _bc.turnright(speed=speed, duration=duration)
        return jsonify(r),200
            


    @bp.route('/admin/restart', methods = ['GET', 'POST'])
    def admin_restart():
        print(f"{os.getcwd()}")
        os.chdir("..")
        os.system("sudo python cb_restart_all.py &")
        return jsonify({"OK": True}),200

    return bp