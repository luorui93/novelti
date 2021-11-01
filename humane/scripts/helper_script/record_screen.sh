#! /bin/bash 
#record screen

filename=$1

#!/bin/bash

handler()
{
    echo quit | nc localhost 8082
}

trap handler SIGINT

vlc \
screen:// \
-I dummy  \
--extraintf rc \
--rc-host localhost:8082 \
--no-video :screen-fps=24 :screen-caching=300 \
--sout "#transcode{vcodec=h264,vb=8192,acodec=mp3,ab=192,fps=24,scale=1}:duplicate{dst=std{access=file,mux=mp4,dst='$filename'}}" \
2>/dev/null

#--rc-quiet \
#--dummy-quiet
#--screen-follow-mouse \
#--screen-mouse-image="mouse_pointer.png" \
#--screen-left=0 --screen-top=0 --screen-width=800 --screen-height=600 \
