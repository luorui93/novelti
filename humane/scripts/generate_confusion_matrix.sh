#! /bin/bash
# Parameters:
# cmd_mapping_file: the location of user's emotiv expression mapping file
# confidence_threshold: the threshold of emotiv commander
# decision_time: the period user has to determine the input
# measurement_time the duration emotiv will measure user input
if [ "$#" -eq 1 ];
then 
    user=$1

    exp_times=5
    cmd_mapping_file="$HOME/humane_data/emotiv_cmd_mapping/${user}_mapping.yaml"
    inputSource="controlpanel"   #controlpanel, composer

    emotiv_config_file="$HOME/anna_ws/src/humane/config/emotiv_config.yaml"

    SCRIPTDIR=$(cd `dirname $0` && pwd)
    datadir="$HOME/humane_data"
    timestamp=$(date +%Y_%m_%d_%H_%M_%S)
    filename="confusion_matrix_build-$user-$timestamp"

    record_webcam="$SCRIPTDIR/helper_script/record_webcam.sh $datadir/webcam/$filename.mp4"
    record_screen="$SCRIPTDIR/helper_script/record_screen.sh $datadir/screen/$filename.mp4"

    echo "$record_webcam & $record_screen &"
    $record_webcam & 
    $record_screen &

    roslaunch emotiv_epoc_driver emotiv_commander.launch confusion_matrix_builder:=true cm_repeat_times:=$exp_times inputSource:=$inputSource cm_filepath:=$HOME/humane_data/confusion_matrix/${user}_emotiv.yaml cmd_mapping_file:=$cmd_mapping_file emotiv_config_file:=$emotiv_config_file

    killall ffmpeg
    killall vlc

    echo "Confusion matrix generated!"
else
    echo -e "\e[31mNot enough arguments, use following pattern:"
    echo -e "\e[33m$0 user_name"
    echo -e "\e[0m"
fi
