#!/usr/bin/env python3
import os
import subprocess
from pathlib import Path
import sys

def run_bash(script_name: str, script_dir: Path) -> subprocess.Popen:
    script_path = script_dir / script_name
    if not script_path.exists():
        print(f"[ERR] script not found: {script_path}", file=sys.stderr)
        return None
    # Optional: ensure LF endings and executable
    # (run once manually: sed -i 's/\r$//' <script> && chmod +x <script>)
    log_dir = script_dir / "logs"
    log_dir.mkdir(exist_ok=True)
    stdout_f = open(log_dir / f"{script_name}.out.log", "ab", buffering=0)
    stderr_f = open(log_dir / f"{script_name}.err.log", "ab", buffering=0)
    print(f"[RUN] {script_path}")
    return subprocess.Popen(
        ["/bin/bash", str(script_path)],
        cwd=str(script_dir),
        stdout=stdout_f,
        stderr=stderr_f,
        start_new_session=True,   # detach from this process group
        env=os.environ.copy(),
    )

def main():
    # Base directory = folder containing this Python file
    script_dir = Path(__file__).resolve().parent
    # If on Raspberry Pi, optionally change to /home/ws if your scripts live there
    if "raspberrypi" in os.uname().nodename.lower() or "raspberrypi" in os.uname().machine.lower():
        # Prefer absolute paths over chdir; but if your scripts are in /home/ws, set script_dir accordingly
        if (Path("/home/ws") / "cb_subsystem_ros2_nav.sh").exists():
            script_dir = Path("/home/ws")

    print(f"[CWD] {os.getcwd()}")
    print(f"[DIR] script_dir={script_dir}")

    p1 = run_bash("cb_subsystem_ros2_nav.sh", script_dir)
    p2 = run_bash("cb_subsystem_robot_terminal_server.sh", script_dir)

    if p1 is None or p2 is None:
        print("[WARN] one or more scripts failed to launch (see logs).")

    # Keep this supervisor alive so children aren’t reaped by a service manager
    try:
        pids = [(name, p.pid) for name, p in (("ros2_nav", p1), ("robot_term", p2)) if p is not None]
        print(f"[PIDS] {pids}")
        # Wait forever; or replace with a simple health check loop
        p1_wait = p1.wait if p1 is not None else lambda: None
        p2_wait = p2.wait if p2 is not None else lambda: None
        p1_wait(); p2_wait()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()