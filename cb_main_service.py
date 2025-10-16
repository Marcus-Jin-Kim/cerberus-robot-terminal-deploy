#!/usr/bin/env python3
import atexit
import os
import signal
import subprocess
from pathlib import Path
import sys, time, socket, urllib.parse
import urllib.request
import yaml
import json

_SKYNET_SERVER_HOSTNAMES_ = ["192.168.137.1", "192.168.0.20", "192.168.77.7"]

# _CONST_CB_ROBOT_TYPE_UGV_RPI_ = "UGV_RPI"
# _CONST_CB_ROBOT_TYPE_UGV_JETSON =  "UGV_JETSON"
# _CONST_ROS2_CONTAINER_UGV_RPI_ = "ugv_rpi_ros_humble"
# _CONST_ROS2_CONTAINER_UGV_JETSON_ = "ugv_jetson_ros_humble"
# _CONST_HOST_HOME_DIR_UGV_RPI_ = "/home/ws"
# _CONST_HOST_HOME_DIR_UGV_JETSON_ = "/home/jetson"
# _CONST_DOCKER_BRINGUP_MAIN_SCRIPT_FILENAME_RPI_ = "cb_bringup_main_rpi.sh"
# _CONST_DOCKER_BRINGUP_MAIN_SCRIPT_FILENAME_JETSON_ = "cb_bringup_main_jetson.sh"

# TEMP until gets a server
___DEV_TEST_ROBOT_UID_ = "beast001"

def init_my_robot_config():

    machine_id = ""
    robot_os = ""

    robot_chassis = ""
    robot_type = ""
    host_home_dir = ""
    robot_uid = ""
    ros2_container_name = ""

    try:
        if os.path.exists("/home/jetson") and os.path.exists("/home/jetson/ugv_jetson"):
            robot_os = "JETSON"

            ugv_config = yaml.safe_load(open("/home/jetson/ugv_jetson/config.yaml", 'r'))
            robot_chassis = ugv_config.get("base_config").get("robot_name")

        elif os.path.exists("/home/ws") and os.path.exists("/home/ws/ugv_rpi"):
            robot_os = "RPI"

            ugv_config = yaml.safe_load(open("/home/ws/ugv_rpi/config.yaml", 'r'))
            robot_chassis = ugv_config.get("base_config").get("robot_name")

        with open("/etc/machine-id", "r") as f:
            machine_id = f.read().strip()

        robot_config = None
        for hostname in _SKYNET_SERVER_HOSTNAMES_:

            try:
                query = urllib.parse.urlencode({
                    "robot-os": robot_os or "",
                    "robot-chassis": robot_chassis or ""
                })
                mid_q = urllib.parse.quote(machine_id, safe="")
                url = f"http://{hostname}:8000/init-my-robot-config-mid/{mid_q}?{query}"
                print(f"[QUERY] {url}")
                with urllib.request.urlopen(url, timeout=5) as response:
                    if response.status == 200:
                        resp_json = response.read().decode('utf-8')        
                        resp = json.loads(resp_json)
                        if resp.get("OK") and "robot_config" in resp:
                            robot_config = resp["robot_config"]
                            robot_config["SKYNET_SERVER_HOST"] = hostname
                            robot_config["SKYNET_SERVER_URL_ROOT"] = f"http://{hostname}:8000"

                            break
            
            except Exception as e:
                print(f"[WARN] cannot query master server at {hostname}: {e}", file=sys.stderr)
                continue
        
        if robot_config is None:
            raise Exception(f"Cannot find robot config from any master server {_SKYNET_SERVER_HOSTNAMES_}")
            return None

        print(f"[INFO] got robot_config for machine_id: {machine_id}")
        # write down to local file

        return robot_config


    except Exception as e:
        print(f"[ERROR] cannot determine robot config: {e}", file=sys.stderr)        
        return None

    return None

def generate_domain_bridge_config(robot_uid, robot_domain_id):
    domain_bridge_config = {
        "from_domain": int(robot_domain_id),
        "to_domain": int(0),
        "topics": {
            "tf": {
                "type": "tf2_msgs/msg/TFMessage",
                "remap": f"{robot_uid}/tf"
            },
            "tf_static": {
                "type": "tf2_msgs/msg/TFMessage",
                "remap": f"{robot_uid}/tf_static"
            }
        }
    }

    return domain_bridge_config
    


def run_bash(script_name: str, params: list, script_dir: Path, debug=False) -> subprocess.Popen:
    script_path = script_dir / script_name
    if not script_path.exists():
        print(f"[ERR] script not found: {script_path}", file=sys.stderr)
        return None
    # Optional: ensure LF endings and executable
    # (run once manually: sed -i 's/\r$//' <script> && chmod +x <script>)
    log_dir = script_dir / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())

    # rotate log using python logging module


    stdout_f = subprocess.DEVNULL
    if debug:
        stdout_f = open(log_dir / f"{script_name}.{stamp}.out", "ab", buffering=0)
    stderr_f = open(log_dir / f"{script_name}.{stamp}.err", "ab", buffering=0)


    print(f"[RUN] {script_path}, {params}")
    return subprocess.Popen(
        ["/bin/bash", str(script_path), *params],
        cwd=str(script_dir),
        stdout=stdout_f,
        stderr=stderr_f,
        start_new_session=True,   # detach from this process group
        env=os.environ.copy(),
    )

def check_docker(_try=30) -> bool:    
    t = 0
    while t < _try:
        r = os.system("systemctl is-active --quiet docker; echo $?")
        if r == 0:
            return True
        t += 1
        time.sleep(1)
    return False


def main(_get_config_max_retry = 30, start_ros=True, start_terminal=True):  # 3 min retry
    # Base directory = folder containing this Python file
    # script_dir = Path(__file__).resolve().parent
    # If on Raspberry Pi, optionally change to /home/ws if your scripts live there

    robot_config = None
    get_config_try = 0
    while robot_config is None and get_config_try < _get_config_max_retry:
        robot_config = init_my_robot_config()
    
    
    if robot_config is None:
        print(f"[ERR] cannot get robot config after {_get_config_max_retry} tries; exiting.", file=sys.stderr)
        return 1
        
    # switch user to robot_os_user_id
    robot_os_user_id = ""
    if robot_config.get("ROBOT_OS") == "RPI":
        robot_os_user_id = "ws"
    elif robot_config.get("ROBOT_OS") == "JETSON":
        robot_os_user_id = "jetson"
    else:
        print(f"[ERR] unknown ROBOT_OS {robot_config.get('ROBOT_OS')}; cannot switch user", file=sys.stderr)
        return 1

    # set uid is working but now i have permission problem
    # try:
    #     import pwd
    #     user_info = pwd.getpwnam(robot_os_user_id)
    #     os.setgid(user_info.pw_gid)
    #     os.setuid(user_info.pw_uid)
    #     os.environ['HOME'] = user_info.pw_dir
    #     print(f"[USER] switched to user {robot_os_user_id}")
    # except Exception as e:
    #     print(f"[ERR] cannot switch to user {robot_os_user_id}: {e}", file=sys.stderr)
    #     return 1
    
    # write down to local file
    with open("cb_config.yaml", "w") as f:
        yaml.safe_dump(robot_config, f)
        print(f"[INFO] wrote local config to ./cb_config.yaml")


    robot_type = robot_config.get("ROBOT_TYPE")
    host_home_dir = robot_config.get("HOST_HOME_DIR")
    robot_maker_script_dir = robot_config.get("ROBOT_MAKER_SCRIPT_DIR")
    docker_home_dir = robot_config.get("DOCKER_HOME_DIR")
    ros2_container_name = robot_config.get("ROS2_CONTAINER_NAME")
    docker_script_filename = robot_config.get("DOCKER_BRINGUP_MAIN_SCRIPT_FILENAME")
    robot_uid = robot_config.get("ROBOT_UID")
    initial_pose_x = str(float(robot_config.get("ROBOT_START_POSITION_AND_YAW", [0,0,0])[0])).format('.2f')
    initial_pose_y = str(float(robot_config.get("ROBOT_START_POSITION_AND_YAW", [0,0,0])[1])).format('.2f')
    initial_pose_yaw = str(float(robot_config.get("ROBOT_START_POSITION_AND_YAW", [0,0,0])[2])).format('.2f')
    
    print(f"[TYPE] robot_type={robot_type} host_home_dir={host_home_dir} ros2_container={ros2_container_name} robot_uid={robot_uid}")

    
    host_script_dir = Path(f"{host_home_dir}/cb")
    docker_script_dir = Path(f"{docker_home_dir}/cb")
    docker_script_path = f"{docker_script_dir}/cb_docker_tools/{docker_script_filename}"

    robot_domain_id = robot_config.get("ROBOT_DOMAIN_ID")

    with open("cb_domain_bridge_config.yaml", "w") as f:
        domain_bridge_config = generate_domain_bridge_config(robot_uid, robot_domain_id)
        yaml.safe_dump(domain_bridge_config, f)
        print(f"[INFO] wrote domain bridge config to ./cb_domain_bridge_config.yaml")

    # DIRTY HACK TO ENSURE SCRIPT IS EXECUTABLE
    os.chmod(f"{host_home_dir}/cb/cb_docker_tools/{docker_script_filename}", 0o755)
    # os.chmod(docker_script_path, 0o755)  # ensure executable
    
    # if ROBOT_TYPE == _CONST_CB_ROBOT_TYPE_UGV_RPI_:
    #     # Prefer absolute paths over chdir; but if your scripts are in /home/ws, set script_dir accordingly
    #     os.chdir(_CONST_HOST_HOME_DIR_UGV_RPI_)
    #     script_dir = Path(f"{_CONST_HOST_HOME_DIR_UGV_RPI_}/cb")
    # elif robot_type == _CONST_CB_ROBOT_TYPE_UGV_JETSON:
    #     os.chdir(_CONST_HOST_HOME_DIR_UGV_JETSON_)
    #     script_dir = Path(f"{_CONST_HOST_HOME_DIR_UGV_JETSON_}/cb")
    # else:
    #     raise Exception(f"Unsupported robot type: {robot_type}")

    print(f"[DIR] host script_dir={host_script_dir}")
    os.chdir(host_script_dir)
    print(f"[CWD] {os.getcwd()}")
    
    if start_ros:
        print("[DOCK] starting ros2 docker..")
        print("[DOCK] checking docker service is running...")
        if not check_docker():
            print("[ERR] docker does not appear to be running; cannot continue.", file=sys.stderr)
            return 1
        
        print(f"[DOCK] starting docker container {ros2_container_name}...")
        time.sleep(5) # just give some time
        r = os.system(f"docker start {ros2_container_name}")
        if r != 0:
            print(f"[ERR] failed to start docker container {ros2_container_name}", file=sys.stderr)
            raise Exception(f"failed to start docker container {ros2_container_name}")
        
        time.sleep(5) # just give some time

        print(f"[DOCK] starting subsystems...")

        # build ros2 docker params
        docker_params = [
            ros2_container_name,
            docker_script_path,
            robot_uid,
            str(robot_domain_id),
            str(initial_pose_x),
            str(initial_pose_y),
            str(initial_pose_yaw)
        ]

        p1 = run_bash(f"cb_subsystem_ros2_nav.sh", docker_params, host_script_dir)
        if p1 is None:        
            print("[ERROR] failed to launch ros2_nav subsystem", file=sys.stderr)        
            raise Exception("failed to launch ros2_nav subsystem")

    if start_terminal:
        # start terminal server
        print(f"[TERM] starting terminal server...")
         # build terminal server params
        terminal_server_params = [
            robot_maker_script_dir
        ]    

        # p2 = run_bash("ls", script_dir)  # replace with actual script
        p2 = run_bash("cb_subsystem_terminal_server.sh", terminal_server_params, host_script_dir)


        if p2 is None:
            print("[ERROR] failed to launch terminal_server subsystem", file=sys.stderr)        
            raise Exception("failed to launch terminal_server subsystem")
    
    print(f"[INFO] launched subsystems. exiting main service")
    # print(f"[INFO] launched subsystems, waiting forever...")
    # try:
    #     # i just want to keep this process alive, no need to wait on the subprocesses
    #     while True:
    #         time.sleep(10)
    # except KeyboardInterrupt:
    #     print("[EXIT] KeyboardInterrupt received; exiting.")
    #     pass

if __name__ == "__main__":
    ## main service no longer holds terminal
    start_ros = True
    start_terminal = True
    arg = ""
    if len(sys.argv) >= 2:
        for arg in sys.argv[1:]:
            if arg == "ros-only":
                start_ros = True
                start_terminal = False
            elif arg == "terminal-only":
                start_ros = False
                start_terminal = True            
            else:
                print(f"[WARN] unknown argument '{arg}'; ignoring.", file=sys.stderr)

    print(f"[INFO] starting services: ROS={start_ros}, Terminal={start_terminal}")
    main(start_ros=start_ros, start_terminal=start_terminal)

    # # write down pid and ensure cleanup
    # with open("cb_pid_main_service.txt", "w") as f:
    #     f.write(str(os.getpid()) + "\n")
    #     print(f"[SERV] wrote PID {os.getpid()} to cb_pid_main_service.txt")

    # def _cleanup_pidfile():
    #     if os.path.exists("cb_pid_main_service.txt"):
    #         os.remove("cb_pid_main_service.txt")
    #         print("[SERV] main service PID file removed.")
    # def _on_signal(signum, frame):
    #     print(f"[SERV] signal {signum} received; exiting")
    #     _cleanup_pidfile()
    #     sys.exit(0)
    # atexit.register(_cleanup_pidfile)    
    # signal.signal(signal.SIGTERM, _on_signal)
    # signal.signal(signal.SIGINT, _on_signal)

