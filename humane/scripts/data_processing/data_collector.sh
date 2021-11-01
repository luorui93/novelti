#! /bin/bash

DATADIR=$HOME/humane_data

if [ "$#" -eq 1 ]
then
    filename=$1
    mkdir $DATADIR/organized/$filename/
    #check if screen record exists
    if [ -f $DATADIR/screen/$filename.mp4 ]
    then
        cmd="cp $DATADIR/screen/$filename.mp4 $DATADIR/organized/$filename/screen.mp4"
        $cmd
        echo copied screen record
    else
        echo "screen record file for $1 is not found"
    fi
    #check if webcam record exists
    if [ -f $DATADIR/webcam/$filename.mp4 ]
    then
        cmd="cp $DATADIR/webcam/$filename.mp4 $DATADIR/organized/$filename/webcam.mp4"
        $cmd
        echo copied webcam record
    else
        echo "webcam record file for $1 is not found"
    fi
    #check if bag file exists
    if [ -f $DATADIR/bag/$filename.bag.tar.gz ]
    then
        #cmd="cp $DATADIR/bag/$filename.bag.tar.gz $DATADIR/organized/$filename/bagfile.bag.tar.gz"
        #$cmd
        tar xzvf $DATADIR/bag/$filename.bag.tar.gz -C $DATADIR/organized/$filename
        echo extracted bag file
    else
        echo "bag file for $1 is not found"
    fi
    #check if assessment file exists
    if [ -f $DATADIR/assessment/$filename.csv ]
    then
        cmd="cp $DATADIR/assessment/$filename.csv $DATADIR/organized/$filename/assessment.csv"
        $cmd
        echo copied assesment
    else
        echo "assessment file for $1 is not found"
    fi
else
    echo "Please specify data file name"
fi