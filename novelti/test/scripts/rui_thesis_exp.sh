#!/bin/bash

start=rs1   #rs1, rs2, rs3 corresponding to path0,1,2
path=0 #path = 0,1,2

subject_name=test
pos_method=nearcog_obst  #nearcog_obst, no_move
div_method=extredist  #extredist, altertile
mx=mx100 #mx100 mx91 mx85 mx70
os_strategy=opt #stll opt
record=1

mkdir /home/yaphes/bag/$subject_name
cmd="roslaunch novelti novelti.launch start_pose_name:=$start pos:=$pos_method div:=$div_method mx:=$mx bag:=$record bagpath:=/home/yaphes/bag/$subject_name/ os_method:=$os_strategy path:=$path"

echo "Starting subject experiment..."
echo "Running: '$cmd'"
$cmd