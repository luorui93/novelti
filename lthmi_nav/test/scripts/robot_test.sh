#!/bin/bash

intent=debugging    #experiment
start=office1
dst=bathroom1
pos=nearcog_obst
div=extredist
mx=mx85
use_pois=0
ksafe=1.3
time_start=`date +"%Y-%m-%d_%H-%M-%S_%Z%z"`

cmd="echo Echoing start:=$start dst:=$dst pos:=$pos div:=$div use_pois:=$use_pois"
#cmd="roslaunch lthmi_nav key.launch start:=$start dst:=$dst pos:=$pos div:=$div use_pois:=$use_pois ksafe:=$ksafe"

time_end=`date +"%Y-%m-%d_%H-%M-%S_%Z%z"`

$cmd

{
    echo "run_intent:   $intent"
    echo "start_loc:    $start"
    echo "dest_loc:     $dst"
    echo "pos:          $pos"
    echo "mx            $mx"
    echo "use_pois:     $use_pois"
    echo "ksafe:        $ksafe"
    echo "time_start:   '$time_start'"
    echo "time_end      '$time_end'"
    echo "result:       '???'"
    echo 
    echo
}>> ~/Desktop/cps/alden-hall-protocol.txt
