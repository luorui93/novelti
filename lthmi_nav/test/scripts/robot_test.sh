#!/bin/bash

intent="recording"    #experiment
start=livroom1     # door1   livroom1   office1   bathroom1   music1   bedroom1   door2
dst=office1
pos=nearcog_obst
div=nearcog_extremal
mx=mx94
use_pois=0
ksafe=1.3
smooth_rads="[]"
view_sizes="[16,32,64,128,256]"
iarea_k=0.9



time_start=`date +"%Y-%m-%d_%H-%M-%S_%Z%z"`

#cmd="echo Echoing start:=$start dst:=$dst pos:=$pos div:=$div use_pois:=$use_pois"
cmd="roslaunch lthmi_nav key.launch  start:=$start  dst:=$dst  pos:=$pos  div:=$div  use_pois:=$use_pois  ksafe:=$ksafe  mx:=$mx  view_sizes:=$view_sizes  smooth_rads:="$smooth_rads"  iarea_k:=$iarea_k"

echo "Running: '$cmd'"
$cmd

time_end=`date +"%Y-%m-%d_%H-%M-%S_%Z%z"`
machine_id=`cat /var/lib/dbus/machine-id`
if [ "$machine_id" == "73fb7affe19b2f79f0bc720600000002"  ]; then
    machine="y580"
elif [ "$machine_id" == "4faf6687b5689ea02e709d9800000002" ]; then
    machine="amazon"
else
    machine="other"
fi
git_commit=`git log --format='%H' -n 1`

{
    echo "run_intent:   $intent"
    echo "start_loc:    $start"
    echo "dest_loc:     $dst"
    echo "pos:          $pos"
    echo "div:          $div"
    echo "mx            $mx"
    echo "use_pois:     $use_pois"
    echo "ksafe:        $ksafe"
    echo "intrstarea_k: $iarea_k"
    echo "smooth_rads:  $smooth_rads"
    echo "view_sizes:   $view_sizes"
    echo "machine:      $machine"
    echo "git_commit:   $git_commit"
    echo "time_start:   '$time_start'"
    echo "time_end      '$time_end'"
    echo "command:      '$cmd'"
    echo "result:       '???'"
    echo 
    echo
}>> ~/Desktop/cps/alden-hall-protocol.txt
