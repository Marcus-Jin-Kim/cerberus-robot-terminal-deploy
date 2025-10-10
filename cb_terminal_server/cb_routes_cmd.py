import os
from flask import Blueprint, Response, jsonify, request
import time
from typing import TYPE_CHECKING
# This import will only run for type checkers, not at runtime
if TYPE_CHECKING:
    from cb_terminal_server import CerberusRobotTerminalServer

## MAX DURATION IS 3 sec
# https://www.waveshare.com/wiki/08_Sub-controller_JSON_Command_Set#CMD_HEART_BEAT_SET

def create_routes_blueprint_cmd(robot_control_server:"CerberusRobotTerminalServer"):

    bp = Blueprint("routes_cmd", __name__)
    _bc = robot_control_server.robot_control.body_control
    # _config = robot_control_server.robot_control.config



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
    @bp.route('/move-vsws', methods = ['GET', 'POST'])
    def move_vsws():
        vspeed = request.args.get('vspeed', default=0, type=float)
        aspeed = request.args.get('aspeed', default=0, type=float)
        duration = request.args.get('duration', default=-1, type=float) # -1 means default duration, 0 means continuous        
        _bc.direct_speed_control(vspeed=vspeed, aspeed=aspeed, duration=duration)
        return jsonify({"OK": True}),200
    
    
    @bp.route('/move-vlwl', methods = ['GET', 'POST'])
    def move_vlwl():
        """ move with vertical and angular LEVEL """
        vl = request.args.get('vl', default=0, type=int)
        wl = request.args.get('wl', default=0, type=int)
        duration = request.args.get('duration', default=-1, type=float) # -1 means default duration, 0 means continuous
        vwd = _config_vlwl_to_dict()

        vlm = 0 if (vl==0) else abs(vl)
        wlm = 0 if (wl==0) else abs(wl)
        vls = 0 if (vl==0) else vl / vlm
        wls = 0 if (wl==0) else wl / wlm

        vs = vwd["vl"][vlm] * vls
        ws = vwd["wl"][wlm] * wls
        _bc.direct_speed_control(vspeed=vs, aspeed=ws, duration=duration)
        return jsonify({"OK": True}),200

    def _config_vlwl_to_dict():
        c = robot_control_server.config
        return {
            "vl": [c["BODY_VL_RATIO_L0"], c["BODY_VL_RATIO_L1"], c["BODY_VL_RATIO_L2"], c["BODY_VL_RATIO_L3"], c["BODY_VL_RATIO_L4"]],
            "wl": [c["BODY_WL_RATIO_L0"], c["BODY_WL_RATIO_L1"], c["BODY_WL_RATIO_L2"], c["BODY_WL_RATIO_L3"], c["BODY_WL_RATIO_L4"]],
        }


    # wrapper routes for general body control
    @bp.route('/forward-slow', methods=['GET','POST'])
    def forward_slow():    
        duration = request.args.get('duration', default=-1, type=float)
        _bc.forward_slow(duration=duration)    
        return jsonify({"OK": True}),200
    
    @bp.route('/forward-cruise', methods=['GET','POST'])
    def forward_cruise():    
        duration = request.args.get('duration', default=-1, type=float)
        _bc.forward_cruise(duration=duration)    
        return jsonify({"OK": True}),200    

    @bp.route('/forward-fast', methods=['GET','POST'])
    def forward_fast():    
        duration = request.args.get('duration', default=-1, type=float)
        _bc.forward_fast(duration=duration)    
        return jsonify({"OK": True}),200

    @bp.route('/back-slow', methods = ['GET', 'POST'])
    def back_slow():    
        duration = request.args.get('duration', default=-1, type=float)
        _bc.back_slow(duration=duration)    
        return jsonify({"OK": True}),200

    @bp.route('/back-cruise', methods = ['GET', 'POST'])
    def back_cruise():
        duration = request.args.get('duration', default=-1, type=float)
        _bc.back_cruise(duration=duration)
        return jsonify({"OK": True}),200

    @bp.route('/back-fast', methods = ['GET', 'POST'])
    def back_fast():
        duration = request.args.get('duration', default=-1, type=float)
        _bc.back_fast(duration=duration)
        return jsonify({"OK": True}),200
    
    @bp.route('/turnleft', methods = ['GET', 'POST'])
    def turnleft():
        duration = request.args.get('duration', default=-1, type=float)
        reverse = request.args.get('reverse', default=0, type=int) # 0 means normal / false , 1 means reverse / true
        _bc.turnleft(duration=duration, reverse=reverse!=0)
        return jsonify({"OK": True}),200

    @bp.route('/turnright', methods = ['GET', 'POST'])
    def turnright():
        duration = request.args.get('duration', default=-1, type=float)
        reverse = request.args.get('reverse', default=0, type=int) # 0 means normal / false , 1 means reverse / true
        _bc.turnright(duration=duration, reverse=reverse!=0)
        return jsonify({"OK": True}),200
    
    @bp.route('/pivotleft', methods = ['GET', 'POST'])
    def pivotleft():
        duration = request.args.get('duration', default=-1, type=float)
        _bc.pivotleft(duration=duration)
        return jsonify({"OK": True}),200

    @bp.route('/pivotright',  methods = ['GET', 'POST'])
    def pivotright():
        duration = request.args.get('duration', default=-1, type=float)
        _bc.pivotright(duration=duration)
        return jsonify({"OK": True}),200

    return bp