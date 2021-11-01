#!/bin/bash

base_sim=0

cmd="roslaunch anna main.launch base_sim:=$base_sim loc:=1 map:=1 auto:=0"

echo "Start mapping.."
echo "Running: '$cmd'"
$cmd