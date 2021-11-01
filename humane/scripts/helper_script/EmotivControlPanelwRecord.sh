#!/bin/bash
filename=$1

./helper_script/record_screen.sh $HOME/humane_data/screen/$filename.mp4 &
./helper_script/record_webcam.sh $HOME/humane_data/webcam/$filename.mp4 &

cd $HOME/EmotivResearch_1.0.0.5
export LD_LIBRARY_PATH=$HOME/EmotivResearch_1.0.0.5/lib
export FONTCONFIG_PATH=/etc/fonts
./EmotivControlPanel &>/dev/null