import datetime
import os
from flask import Blueprint, Response, jsonify, request
import time
import yaml
from typing import TYPE_CHECKING
# This import will only run for type checkers, not at runtime
if TYPE_CHECKING:
    from cb_terminal_server import CerberusRobotTerminalServer

## MAX DURATION IS 3 sec
# https://www.waveshare.com/wiki/08_Sub-controller_JSON_Command_Set#CMD_HEART_BEAT_SET

def create_routes_blueprint_dev(robot_control_server:"CerberusRobotTerminalServer"):

    bp = Blueprint("routes_dev", __name__)
    _bc = robot_control_server.robot_control.body_control
    # _config = robot_control_server.robot_control.config

    @bp.route("/")
    def index():
        if not robot_control_server.robot_control.low_level_control.has_base_control_low_initialized:
            return jsonify({"OK": False, "error": "Low level control not initialized"}), 200            
        else:
            return jsonify({"OK": True, "msg": "Cerberus Robot Controller Terminal Server"}), 200

    @bp.route('/admin/restart-all', methods = ['GET', 'POST'])
    def admin_restart():
        print(f"{os.getcwd()}")
        os.chdir("..")
        os.system("sudo python cb_restart_all.py &")
        return jsonify({"OK": True}),200    
    
    @bp.route('/admin/restart-terminal', methods = ['GET', 'POST'])
    def admin_restart_terminal():
        print(f"{os.getcwd()}")
        os.chdir("..")
        os.system("sudo python cb_restart_all.py restart-terminal &")
        return jsonify({"OK": True}),200    
    


    # @bp.route("/feedback")
    # def feedback():
    #     data = _bc.base.base_control_low.feedback_data()
    #     return jsonify({"OK": True, "data": data}), 200

    @bp.route('/t', methods = ['GET', 'POST'])
    def t():
        qsize = _bc.base.base_control_low.command_queue.qsize()   
        return jsonify({"OK": True, "queue_size": qsize}),200
    
    @bp.route('/cur-config', methods = ['GET', 'POST'])
    def cur_config():
        return jsonify({"OK": True, "config": robot_control_server.config}),200
    
    @bp.route('/set-config-param', methods = ['GET', 'POST'])
    def set_config_param():
        param = request.args.get('p', default=None, type=str)
        value = request.args.get('v', default=None, type=str)
        save_to_file = request.args.get('save', default=1, type=int) # 0 means no, 1 means yes

        if param is None or value is None:
            return jsonify({"OK": False, "error": "param and value are required"}),400
        if param not in robot_control_server.config:
            return jsonify({"OK": False, "error": f"param {param} not found in config"}),400
        
        # try to convert to int or float if possible
        try:
            if '.' in value:
                value = float(value)
            else:
                value = int(value)
        except ValueError:
            pass # keep as string
        robot_control_server.config[param] = value
        if save_to_file:
            # save to config for debugging
            datetime_str = time.strftime("%Y%m%d-%H%M%S")
            filename = f"cb_config_dev_{datetime_str}.yaml"
            with open(filename, "w") as f:
                yaml.dump(robot_control_server.config, f)
            print(f"[INFO] Saved config to {filename}")

        return jsonify({"OK": True, "config": robot_control_server.config}),200
    

    return bp