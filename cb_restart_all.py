#!/usr/bin/env python3
import subprocess, signal
import os, signal
import time
import yaml



def load_cb_config(config_file="cb_config.yaml"):
    if os.path.exists(config_file):
        with open(config_file, 'r') as f:
            cb_config = yaml.safe_load(f)
            print(f"[INFO] Loaded config from {config_file}")
    else:
        raise FileNotFoundError(f"Config file {config_file} not found.")
    return cb_config

def docker_stop(container_name=None):
    r = os.system(f"docker stop {container_name}")
    if r != 0:
        print(f"[WARN] Failed to stop container {container_name} or it was not running.")
    else:
        print(f"[INFO] Stopped container {container_name}.")





if __name__ == "__main__":

    arg = "restart"
    if len(os.sys.argv) > 1:
        arg = os.sys.argv[1]

    cb_config = load_cb_config()    
    container_name = cb_config.get("ROS2_CONTAINER_NAME")

    if arg == "restart":
        print("[INFO] Restarting all subsystems ...")
    elif arg == "stop":
        print("[INFO] Stopping all subsystems ...")
    else:
        print(f"[ERROR] Unknown argument '{arg}'; use 'restart' or 'stop'.")
        exit(1)

    docker_stop(container_name=container_name)

    # find pid files and kill the terminal server
    term_server_pid_file_path = "cb_pid_terminal.txt"
    if os.path.exists(term_server_pid_file_path):
        with open(term_server_pid_file_path, "r") as f:
            pid = int(f.read().strip())
            print(f"[INFO] Found terminal server PID: {pid}, sending SIGTERM")
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                print(f"[WARN] No process with PID {pid} found.")
            except Exception as e:
                print(f"[ERROR] Failed to kill process {pid}: {e}")
    else:
        print(f"[WARN] PID file {term_server_pid_file_path} not found. Is the terminal server running?")


    time.sleep(5)

    main_service_pid_file_path = "cb_pid_main_service.txt"
    if os.path.exists(main_service_pid_file_path):
        with open(main_service_pid_file_path, "r") as f:
            pid = int(f.read().strip())
            print(f"[INFO] Found main service PID: {pid}, sending SIGTERM")
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                print(f"[WARN] No process with PID {pid} found.")
            except Exception as e:
                print(f"[ERROR] Failed to kill process {pid}: {e}")
    else:
        print(f"[WARN] PID file {main_service_pid_file_path} not found. Is the main service running?")

    print("[INFO] kill processes and docker stop completed. Please manually verify all subsystems are down before restarting.")

    if arg == "stop":
        print("[INFO] Stop only as per argument; exiting.")
        exit(0)

    time.sleep(5)
    # os.system("bash -lc 'python3 cb_main_service.py & disown'")

    subprocess.Popen(
        ["python3", "cb_main_service.py"],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True
    )