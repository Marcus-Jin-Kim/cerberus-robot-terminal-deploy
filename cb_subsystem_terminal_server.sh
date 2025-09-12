#!/bin/bash
# add ugv_rpi to PYTHONPATH
# cd /home/ws/cb
export PYTHONPATH="$PYTHONPATH:$(pwd)/../ugv_rpi"
cd cb_terminal_server
source cb-env/bin/activate
# python cb_cmd.py
python cb_terminal_server.py