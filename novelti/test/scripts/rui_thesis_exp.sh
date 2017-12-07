#!/bin/bash

start=office1   # door1   livroom1   office1   bathroom1   music1   bedroom1   door2   kitchen1   storage1   storage2   storage3   livroom2
dst=livroom1

subject_name=rui
div_method=extredist
mx=mx85   #mx100 mx91 mx85 mx70
oc_strategy=still  #stll max_prob intermediate_angle
record=1

mkdir /home/yaphes/bag/$subject_name
cmd="roslaunch novelti novelti.launch start_pose_name:=$start_pose_name div:=$div_method mx:=$mx bag:=$record bagpath:=/home/yaphes/bag/$subject_name/ ori_method:=$oc_strategy"

echo "Starting subject experiment..."
echo "Running: '$cmd'"
$cmd