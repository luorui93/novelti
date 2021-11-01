#! /bin/bash
export ROS_IP=192.168.1.3
export ROS_MASTER_URI=http://192.168.1.2:11311
export DISPLAY=:0

source $HOME/anna_ws/devel/setup.bash
timestamp=$(date +%Y-%m-%d_%H-%M-%S)

mkdir -p $HOME/humane_data/gazebo

$HOME/anna_ws/src/humane/scripts/helper_script/record_screen.sh  $HOME/humane_data/gazebo/$timestamp >> $HOME/humane_data/gazebo/$timestamp.log 2>&1   &

exec "$@"
