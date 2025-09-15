#!/bin/bash
# add ugv_rpi to PYTHONPATH
# cd /home/ws/cb
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <base_dir>" >&2
  exit 1
fi
base_dir="$1"

# export PYTHONPATH="$PYTHONPATH:$(pwd)/../ugv_rpi"
export PYTHONPATH="$PYTHONPATH:$base_dir"
cd cb_terminal_server
source cb-env/bin/activate
# python cb_cmd.py
python cb_terminal_server.py