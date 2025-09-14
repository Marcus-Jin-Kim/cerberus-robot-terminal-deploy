#!/usr/bin/env python3
import os
import subprocess
from pathlib import Path
import sys, time, socket, urllib.parse

_CONST_CB_ROBOT_TYPE_UGV_RPI_ = "UGV_RPI"
_CONST_CB_ROBOT_TYPE_UGV_JETSON =  "UGV_JETSON"
_CONST_ROS2_CONTAINER_UGV_RPI_ = "ugv_rpi_ros_humble"
_CONST_ROS2_CONTAINER_UGV_JETSON_ = "ugv_jetson_ros_humble"
_CONST_HOST_HOME_DIR_UGV_RPI_ = "/home/ws"
_CONST_HOST_HOME_DIR_UGV_JETSON_ = "/home/jetson"
_CONST_DOCKER_BRINGUP_MAIN_SCRIPT_FILENAME_RPI_ = "cb_bringup_main_rpi.sh"
_CONST_DOCKER_BRINGUP_MAIN_SCRIPT_FILENAME_JETSON_ = "cb_bringup_main_jetson.sh"

# TEMP until gets a server
___DEV_TEST_ROBOT_UID_ = "beast001"

def find_robot_type():
    robot_type = ""
    host_home_dir = ""
    robot_uid = ""
    ros2_container_name = ""
    if "raspberrypi" in os.uname().nodename.lower() or "raspberrypi" in os.uname().machine.lower():
        robot_type = _CONST_CB_ROBOT_TYPE_UGV_RPI_
        host_home_dir = _CONST_HOST_HOME_DIR_UGV_RPI_        
        ros2_container_name = _CONST_ROS2_CONTAINER_UGV_RPI_
        docker_script_filename = _CONST_DOCKER_BRINGUP_MAIN_SCRIPT_FILENAME_RPI_
    elif "tegra" in os.uname().nodename.lower() or "tegra" in os.uname().machine.lower():
        robot_type = _CONST_CB_ROBOT_TYPE_UGV_JETSON
        host_home_dir = _CONST_HOST_HOME_DIR_UGV_JETSON_
        ros2_container_name = _CONST_ROS2_CONTAINER_UGV_JETSON_
        docker_script_filename = _CONST_DOCKER_BRINGUP_MAIN_SCRIPT_FILENAME_JETSON_
    else:
        raise Exception(f"Cannot determine robot type from uname: {os.uname()}")

    # read from /etc/machine-id if possible
    machine_id = ""
    try:
        with open("/etc/machine-id", "r") as f:
            machine_id = f.read().strip()
    except Exception as e:
        print(f"[ERROR] cannot read /etc/machine-id: {e}", file=sys.stderr)
        raise e
    # TODO QUERY master server for robot_uid later
    
    robot_uid = ___DEV_TEST_ROBOT_UID_

    return robot_type, host_home_dir, ros2_container_name, docker_script_filename, robot_uid

def run_bash(script_name: str, params: list, script_dir: Path) -> subprocess.Popen:
    script_path = script_dir / script_name
    if not script_path.exists():
        print(f"[ERR] script not found: {script_path}", file=sys.stderr)
        return None
    # Optional: ensure LF endings and executable
    # (run once manually: sed -i 's/\r$//' <script> && chmod +x <script>)
    log_dir = script_dir / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
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


def main():
    # Base directory = folder containing this Python file
    # script_dir = Path(__file__).resolve().parent
    # If on Raspberry Pi, optionally change to /home/ws if your scripts live there
    robot_type, host_home_dir, ros2_container_name, docker_script_filename, robot_uid = find_robot_type()
    print(f"[TYPE] robot_type={robot_type} host_home_dir={host_home_dir} ros2_container={ros2_container_name} robot_uid={robot_uid}")

    
    host_script_dir = Path(f"{host_home_dir}/cb")
    docker_script_path = f"{host_script_dir}/cb_docker_tools/{docker_script_filename}"
    
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
        robot_uid
    ]

    p1 = run_bash(f"cb_subsystem_ros2_nav.sh", docker_params, host_script_dir)
    # p2 = run_bash("ls", script_dir)  # replace with actual script
    p2 = run_bash("cb_subsystem_terminal_server.sh", [], host_script_dir)

    if p1 is None or p2 is None:
        print("[WARN] one or more scripts failed to launch (see logs).")

    # Keep this supervisor alive so children aren’t reaped by a service manager
    try:
        pids = [(name, p.pid) for name, p in (("ros2_nav", p1), ("robot_term", p2)) if p is not None]
        # pids = [(name, p.pid) for name, p in (("ros2_nav", p1),) if p is not None]
        print(f"[PIDS] {pids}")
        # Wait forever; or replace with a simple health check loop
        p1_wait = p1.wait if p1 is not None else lambda: None
        p2_wait = p2.wait if p2 is not None else lambda: None
        p1_wait() ; p2_wait()
    except KeyboardInterrupt:
        print("[EXIT] KeyboardInterrupt received; exiting.")
        pass

if __name__ == "__main__":
    main()