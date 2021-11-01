#! /bin/bash

############DELETE THIS FILE##########
if [ "$#" -eq 5 ];
then 
    user=$1
    context=$2
    control=$3
    path=$4
    custom=$5
    timestamp=$(date +%Y-%m-%d_%H-%M-%S)
    control_ui="keyboard"
    emotiv="composer"
    bagdir="$HOME/humane_data/bagfile"
    bagfile="$user-$context-$control-$path-$custom-$timestamp.bag"

    if [ "$control" = "novel" ]; 
    then
        use_novelti="true"
        echo "use novelti"
        use_assistive="false"
        use_auto="true"
    else 
        use_novelti="false"
        echo "use steering control"
        use_assistive="true"
        use_auto="false"
    fi
    cmd="roslaunch humane humane.launch sim:=0 bag:=1 control_ui:=$control_ui use_novelti:=$use_novelti use_assistive:=$use_assistive use_auto:=$use_auto emotiv_source:=$emotiv bag_file:=$bagdir/$bagfile path:=$path"

    echo "run $cmd"
    $cmd
    echo "Compressing bag file..."
    cmd="tar -czvf $bagdir/$bagfile.tar.gz  -C $bagdir $bagfile"
    echo "$cmd"
    $cmd
    rm $bagdir/$bagfile
else
    echo -e "\e[31mNot enough arguments, use following pattern:"
    echo -e "\e[33m$0 user_name sim/rob steer/novel path_name custom_mode"
    echo -e "\e[0m"
fi