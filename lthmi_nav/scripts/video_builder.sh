#!/bin/bash



# sudo sshfs -o allow_other,reconnect -p 2222 git@zzzzzz.no-ip.org:/home/git/ah_data /home/sd/Desktop/ah_data
# sudo killall -9 sshfs
# sudo fusermount -u /home/sd/Desktop/ah_data

data_root=/home/sd/Desktop/ah_data/2016-12-experiments-in-alden-hall
tmp=~/Desktop
outdir=$tmp/1

desktop_file=$data_root/desktop/alden_hall_experiment_desktop_2016-12-28_19-04-03_EST-0500.mp4
desktop_start_time=10
cam_file="$data_root/video/2016-12-(20-29) - runs/HD4K/BDMV/STREAM/00047.MTS"
cam_start_time=910
cam_duration=80

# http://ffmpeg.org/ffmpeg-filters.html

ffmpeg \
    -i $desktop_file -ss $cam_start_time -t $cam_duration  -i "$cam_file"  \
    -filter_complex " \
        nullsrc=size=1920x1178 [base];  \
        [0:v] trim=start=$desktop_start_time, setpts=PTS-STARTPTS, split [map][div];\
        [map] crop=541:777:86:92  [vmap];\
        [div] crop=670:777:824:92 [vdiv];\
        [1:v] crop=1920:400:0:660 [vcam];
        [base][vmap] overlay=shortest=1:x=0:y=0 [tmp1];\
        [tmp1][vdiv] overlay=shortest=1:x=541:y=0 [tmp2];\
        [tmp2][vcam] overlay=shortest=1:x=0:y=777\
    "\
    -metadata title="Robotic wheelchair controlled via a low-throughput human-machine interface" -metadata year="2017" -metadata author="Dmitry A. Sinyukov" -metadata artist="Dmitry A. Sinyukov"\
    -c:v libx264 $outdir/output.mp4

