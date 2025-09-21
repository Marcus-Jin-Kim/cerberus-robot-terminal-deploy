import os
from cb_terminal_server import CerberusRobotTerminalServer
from flask import Blueprint, Response, jsonify, request
import time

## MAX DURATION IS 3 sec
# https://www.waveshare.com/wiki/08_Sub-controller_JSON_Command_Set#CMD_HEART_BEAT_SET

def create_routes_blueprint(robot_control_server:CerberusRobotTerminalServer):

    bp = Blueprint("cmd_routes", __name__)
    _bc = robot_control_server.robot_control.body_control
    # _config = robot_control_server.robot_control.config

    @bp.route("/")
    def index():
        if not robot_control_server.robot_control.low_level_control.has_base_control_low_initialized:
            return jsonify({"OK": False, "error": "Low level control not initialized"}), 200            
        else:
            return jsonify({"OK": True, "msg": "Cerberus Robot Controller Terminal Server"}), 200

    @bp.route('/admin/restart', methods = ['GET', 'POST'])
    def admin_restart():
        print(f"{os.getcwd()}")
        os.chdir("..")
        os.system("sudo python cb_restart_all.py &")
        return jsonify({"OK": True}),200    
    # @bp.route("/feedback")
    # def feedback():
    #     data = _bc.base.base_control_low.feedback_data()
    #     return jsonify({"OK": True, "data": data}), 200

    @bp.route("/stream")
    def stream():
        return Response(robot_control_server.mjpeg_gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @bp.route("/get-mp-pose")
    def get_mp_pose():
        with robot_control_server._lock:
            data = robot_control_server.last_json if robot_control_server.last_json is not None else {"OK": False}
        return jsonify({"OK": True, "data": data}), 200

    @bp.route('/stop', methods = ['GET', 'POST'])
    def stop():
        _bc.stop()
        return jsonify({"OK": True}),200
    

    # low level direct speed control, as cerberus (unity) does
    @bp.route('/direct-speed-control', methods = ['GET', 'POST'])
    def direct_speed_control():
        vspeed = request.args.get('vspeed', default=0, type=float)
        aspeed = request.args.get('aspeed', default=0, type=float)
        duration = request.args.get('duration', default=-1, type=float) # -1 means default duration, 0 means continuous        
        _bc.direct_speed_control(vspeed=vspeed, aspeed=aspeed, duration=duration)
        return jsonify({"OK": True}),200


    # wrapper routes for general body control
    @bp.route('/forward-slow', defaults={'duration': -1}, methods=['GET','POST'])
    @bp.route('/forward-slow/<duration>', methods=['GET','POST'])
    def forward_slow(duration):    
        try:
            duration_f = float(duration)
        except ValueError:
            return jsonify({"OK": False, "error": "duration must be float"}), 400
        _bc.forward_slow(duration=duration_f)    
        return jsonify({"OK": True}),200
    
    @bp.route('/forward-cruise', defaults={'duration': -1}, methods=['GET','POST'])
    @bp.route('/forward-cruise/<duration>', methods=['GET','POST'])
    def forward_cruise(duration):    
        try:
            duration_f = float(duration)
        except ValueError:
            return jsonify({"OK": False, "error": "duration must be float"}), 400
        _bc.forward_cruise(duration=duration_f)    
        return jsonify({"OK": True}),200
    
    @bp.route('/forward-fast', defaults={'duration': -1}, methods=['GET','POST'])
    @bp.route('/forward-fast/<duration>', methods=['GET','POST'])
    def forward_fast(duration):    
        try:
            duration_f = float(duration)
        except ValueError:
            return jsonify({"OK": False, "error": "duration must be float"}), 400
        _bc.forward_fast(duration=duration_f)    
        return jsonify({"OK": True}),200
        
    @bp.route('/back-slow', defaults={'duration':-1}, methods = ['GET', 'POST'])
    @bp.route('/back-slow/<duration>', methods = ['GET', 'POST'])
    def back_slow(duration):

        try:
            duration_f = float(duration)
        except ValueError:
            return jsonify({"OK": False, "error": "duration must be float"}), 400        
        _bc.back_slow(duration=duration_f)
        return jsonify({"OK": True}),200
    
    @bp.route('/back-cruise', defaults={'duration':-1}, methods = ['GET', 'POST'])
    @bp.route('/back-cruise/<duration>', methods = ['GET', 'POST'])
    def back_cruise(duration):
        try:
            duration_f = float(duration)
        except ValueError:
            return jsonify({"OK": False, "error": "duration must be float"}), 400
        _bc.back_cruise(duration=duration_f)
        return jsonify({"OK": True}),200
    
    @bp.route('/back-fast', defaults={'duration':-1}, methods = ['GET', 'POST'])
    @bp.route('/back-fast/<duration>', methods = ['GET', 'POST'])
    def back_fast(duration):
        try:
            duration_f = float(duration)
        except ValueError:
            return jsonify({"OK": False, "error": "duration must be float"}), 400
        _bc.back_fast(duration=duration_f)
        return jsonify({"OK": True}),200

    @bp.route('/turnleft', defaults={'duration': -1, 'reverse': 0}, methods = ['GET', 'POST'])
    @bp.route('/turnleft/<duration>/<reverse>', methods = ['GET', 'POST'])
    def turnleft(duration, reverse):
        try:
            duration_f = float(duration)
            reverse_i = int(reverse)
        except ValueError:
            return jsonify({"OK": False, "error": "duration and reverse must be float"}), 400
        _bc.turnleft(duration=duration_f, reverse=reverse_i!=0)
        return jsonify({"OK": True}),200
    
    @bp.route('/turnright', defaults={'duration': -1, 'reverse': 0}, methods = ['GET', 'POST'])
    @bp.route('/turnright/<duration>/<reverse>', methods = ['GET', 'POST'])
    def turnright(duration, reverse):
        try:
            duration_f = float(duration)
            reverse_i = int(reverse)
        except ValueError:
            return jsonify({"OK": False, "error": "duration and reverse must be float"}), 400
        #if reverse = 0 set parameter true, if not 0 false
        _bc.turnright(duration=duration_f, reverse=reverse_i!=0)
        return jsonify({"OK": True}),200
    
    @bp.route('/pivotleft', defaults={'duration': -1}, methods = ['GET', 'POST'])
    @bp.route('/pivotleft/<duration>', methods = ['GET', 'POST'])
    def pivotleft(duration):
        try:
            duration_f = float(duration)
        except ValueError:
            return jsonify({"OK": False, "error": "duration must be float"}), 400
        _bc.pivotleft(duration=duration_f)
        return jsonify({"OK": True}),200
    
    
    @bp.route('/pivotright', defaults={'duration': -1}, methods = ['GET', 'POST'])
    @bp.route('/pivotright/<duration>', methods = ['GET', 'POST'])
    def pivotright(duration):
        try:
            duration_f = float(duration)
        except ValueError:
            return jsonify({"OK": False, "error": "duration must be float"}), 400
        _bc.pivotright(duration=duration_f)
        return jsonify({"OK": True}),200









    return bp