#!/usr/bin/env python3
import os
import subprocess
from pathlib import Path
import sys, time, socket, urllib.parse

_CONST_CB_ROBOT_TYPE_UGV_RPI_ = "UGV_RPI"
_CONST_CB_ROBOT_TYPE_UGV_JETSON =  "UGV_JETSON"
_CONST_ROS2_CONTAINER_UGV_RPI_ = "ugv_rpi_ros_humble"
_CONST_ROS2_CONTAINER_UGV_JETSON_ = "ugv_jetson_ros_humble"

# global g_robot_type
# global g_ros2_container_name

def find_robot_type() -> str:
    robot_type = ""
    ros2_container_name = ""
    if "raspberrypi" in os.uname().nodename.lower() or "raspberrypi" in os.uname().machine.lower():
        robot_type = _CONST_CB_ROBOT_TYPE_UGV_RPI_
        ros2_container_name = _CONST_ROS2_CONTAINER_UGV_RPI_
    elif "jetson" in os.uname().nodename.lower() or "jetson" in os.uname().machine.lower():
        robot_type = _CONST_CB_ROBOT_TYPE_UGV_JETSON
        ros2_container_name = _CONST_ROS2_CONTAINER_UGV_JETSON_
    else:
        raise Exception(f"Cannot determine robot type from uname: {os.uname()}")

    return robot_type, ros2_container_name

def run_bash(script_name: str, params: str, script_dir: Path) -> subprocess.Popen:
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
    print(f"[RUN] {script_path}")
    return subprocess.Popen(
        ["/bin/bash", str(script_path), params],
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

# def _docker_ping(docker_host=None, timeout=1.0) -> bool:
#     docker_host = docker_host or os.environ.get("DOCKER_HOST", "unix:///var/run/docker.sock")
#     try:
#         if docker_host.startswith("unix://"):
#             path = docker_host[len("unix://"):]
#             s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
#             s.settimeout(timeout)
#             s.connect(path)
#             s.sendall(b"GET /_ping HTTP/1.0\r\nHost: docker\r\n\r\n")
#             data = s.recv(64)
#             s.close()
#             return b"OK" in data
#         elif docker_host.startswith("tcp://"):
#             u = urllib.parse.urlparse(docker_host)
#             host, port = u.hostname, u.port or 2375
#             with socket.create_connection((host, port), timeout=timeout) as s:
#                 s.sendall(b"GET /_ping HTTP/1.0\r\nHost: docker\r\n\r\n")
#                 return b"OK" in s.recv(64)
#         else:
#             return False
#     except Exception:
#         return False
# def _check_docker_once() -> bool:
#     try:
#         result = subprocess.run( ["docker", "info"],
#             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
#         return result.returncode == 0
#     except (subprocess.CalledProcessError, FileNotFoundError):
#         return False

def main():
    # Base directory = folder containing this Python file
    # script_dir = Path(__file__).resolve().parent
    # If on Raspberry Pi, optionally change to /home/ws if your scripts live there
    robot_type, ros2_container_name = find_robot_type()    
    print(f"[TYPE] robot_type={robot_type}")

    if robot_type == _CONST_CB_ROBOT_TYPE_UGV_RPI_:
        # Prefer absolute paths over chdir; but if your scripts are in /home/ws, set script_dir accordingly
        os.chdir("/home/ws/cb")
        script_dir = Path("/home/ws/cb")

    print(f"[CWD] {os.getcwd()}")
    print(f"[DIR] script_dir={script_dir}")

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


    p1 = run_bash(f"cb_subsystem_ros2_nav.sh", ros2_container_name, script_dir)
    # p2 = run_bash("ls", script_dir)  # replace with actual script
    p2 = run_bash("cb_subsystem_terminal_server.sh", "", script_dir)

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