251015:
using cartographer and seperate domain_id per robot
hence only 3 files:
    ugv_ws_patch/ugv_ws/src/ugv_main/ugv_nav/launch/CB_launch_cartographer_localization_only.py
    ugv_ws_patch/ugv_ws/src/ugv_main/ugv_nav/launch/CB_main_launch.py
    ugv_ws_patch/ugv_ws/src/ugv_main/ugv_nav/launch/nav_bringup/CB_bringup_launch_cartographer.launch.py

build
1. launch docker
2. get in
3. cd /home/ws or jetson/ugv_ws
4. colcon build --symlink-install --packages-select ugv_nav
5. source install/setup.bash