#! /bin/bash 
echo "How was the experiment $filename:"
echo "good: The experiment was successful"
echo "ignore: The result should be deleted"
echo "fail: The experiment was failed"
echo "delete: If you want to delete the log file immediately"

read -p "--> " feedback 

log=$datadir/log.txt

if [ "$feedback" == "delete" ]; then 
    rm $HOME/humane_data/bag/$filename.bag.tar.gz
    rm $HOME/humane_data/screen/$filename.mp4
    rm $HOME/humane_data/webcam/$filename.mp4
    #./evaluation.py $filename
else
    echo "$filename" >> $log
    echo "$feedback" >> $log
    echo "" >> $log
    echo "" >> $log
    printf "Do you want to start workload assessment?(y/n)\n"
    read -p "--> " assessment 

    if [ "$assessment" == "y" ]; then
        $HOME/anna_ws/src/humane/scripts/helper_script/evaluation.py $filename
    fi

fi