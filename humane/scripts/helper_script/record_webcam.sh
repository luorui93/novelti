#!/bin/bash
#record webcam with audio 

filename=$1

cmd="ffmpeg -f v4l2 -video_size 640x480 -i /dev/video0 -f alsa -i default -c:v libx264 -preset ultrafast -c:a aac -strict -2 -loglevel quiet $filename"

echo "run $cmd"
$cmd
