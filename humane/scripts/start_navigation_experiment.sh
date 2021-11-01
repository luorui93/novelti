#! /bin/bash
# Arguments:
# user_id           user name will be recorded in bagfile and will be used to determine the confusion matrix file being used. Use 'button' if button device is used
# context           specify whether the experiment is using gazebo or real robot
# control           choose between steering control and novelti control
# path              determine the path used by this experiment
# custom            custom parameters
# timestamp         timestamp when this experiment runs
# emotiv_source     choose emotiv source among composer, controlpanel and emoengine
# use_emotiv        use emotive or not
# datadir           the directory stores humane experiment data
# filename          the filename used to create rosbag, webcam record, screen record
# cmd_mapping_file  the emotiv cmd mapping file used to run experiment

run_gazebo_locally=1
record_video=1  #webcam and desktop

#trap ctrl_c INT

datadir="$HOME/humane_data"

function ctrl_c() {
    echo "** Trapped CTRL-C"
    export filename=$filename
    export datadir=$datadir
    #xterm -hold to stop xterm from auto closing
    xterm -fa 'Monospace' -fs 14 -e "source /home/yaphes/anna_ws/src/humane/scripts/helper_script/log_result.sh" 
    killall ffmpeg
    killall vlc
}



if [ "$#" -eq 6 ]; then 
    user_id=$1
    context=$2
    control=$3
    ui=$4
    path=$5
    custom=$6
    timestamp=$(date +%Y_%m_%d_%H_%M_%S)
    emotiv_source="controlpanel"
    
    if [ "$ui" = emotiv ]
    then
        ./generate_confusion_matrix.sh $user_id
        use_emotiv=1
    else
        echo "test"
        use_emotiv=0
    fi

    filename="$user_id-$context-$control-$ui-$path-$custom-$timestamp"

    SCRIPTDIR=$(cd `dirname $0` && pwd)

    if [ "$record_video" = "1" ]; then
        record_webcam="$SCRIPTDIR/helper_script/record_webcam.sh $datadir/webcam/$filename.mp4"
        record_screen="$SCRIPTDIR/helper_script/record_screen.sh $datadir/screen/$filename.mp4"

        echo "$record_webcam & $record_screen &"
        $record_webcam & 
        $record_screen &
    fi

    if [ "$custom" = "poi" ]; then
        custom_args="pois:=1"
    else 
        custom_args=
    fi

    if [ "$context" = "sim" ]; then
        sim=1
        
        if [ "$run_gazebo_locally" = "1" ]; then
            export GAZEBO_WORLD_FILE="$HOME/anna_ws/src/humane/gazebo_world/neu_tunnel.world"

        else
            export ROS_MASTER_URI=http://192.168.1.2:11311
            export ROS_IP=192.168.1.2
            unset  ROS_HOSTNAME
            
    #         export GAZEBO_MACHINE_HOST=localhost #192.168.1.3
            export GAZEBO_MACHINE_USER=yaphes
            export GAZEBO_MACHINE_PASSWORD="111aaa"
            export GAZEBO_MACHINE_ENV_LOADER="/home/$GAZEBO_MACHINE_USER/anna_ws/src/humane/scripts/helper_script/remote_env_loader.sh"
            export GAZEBO_WORLD_FILE="/home/$GAZEBO_MACHINE_USER/anna_ws/src/humane/gazebo_world/neu_tunnel.world"
        fi
        
    else 
        sim=0
        export ROS_MASTER_URI=http://localhost:11311
        export ROS_HOSTNAME=localhost
    fi

    if [ "$control" = "novel" ]; then
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

    bagdir="$datadir/bag"
    bagfile="$filename.bag"
    
    cmd="roslaunch humane humane.launch user_id:=${user_id} sim:=$sim bag:=1 use_novelti:=$use_novelti use_assistive:=$use_assistive use_auto:=$use_auto emotiv_source:=$emotiv_source use_emotiv:=$use_emotiv bag_file:=$bagdir/$bagfile path:=$path $custom_args" 

    echo "run $cmd"
    $cmd
    echo "Compressing bag file..."
    cmd="tar -czvf $bagdir/$bagfile.tar.gz  -C $bagdir $bagfile"
    echo "$cmd"
    $cmd
    rm $bagdir/$bagfile

    sshpass -p "${GAZEBO_MACHINE_PASSWORD}" ssh "${GAZEBO_MACHINE_USER}@${GAZEBO_MACHINE_HOST}" pkill -f vlc
    ctrl_c
else
    echo -e "\e[31mNot enough arguments, use following pattern:"
    echo -e "\e[33m$0 user_id sim/rob steer/novel emotiv/button path_name custom_mode"
    echo -e "\e[0m"
fi
