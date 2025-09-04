#!/bin/bash
# add ugv_rpi to PYTHONPATH
export PYTHONPATH="$PYTHONPATH:$(pwd)/../ugv_rpi"
source ../cb-env/bin/activate
# python cb_cmd.py
python cb_robot_terminal_server.py