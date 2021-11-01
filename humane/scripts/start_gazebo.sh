#! /bin/bash

if [ "$#" -eq 5 ];
then 
    user=$1
    scene=$2
    control=$3
    path=$4
    custom=$5
    timestamp=$(date +%Y-%m-%d_%H-%M-%S)
    emotiv="none"

    filename="$user-$context-$control-$path-$custom-$timestamp.bag"

    if [ "$control" = "novel" ]; 
    then
        use_novelti="true"
        echo "use novelti"
    else 
        use_novelti="false"
        echo "use steering control"
    fi
    cmd="roslaunch humane anna.launch sim:=1"

    echo "run $cmd"
    $cmd
else
    echo -e "\e[31mNot enough arguments, use following pattern:"
    echo -e "\e[33m$0 user_name sim/rob steer/novel path_name custom_mode"
    echo -e "\e[0m"
fi