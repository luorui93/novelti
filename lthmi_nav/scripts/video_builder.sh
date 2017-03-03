#!/bin/bash

# sudo sshfs -o allow_other,reconnect -p 2222 git@zzzzzz.no-ip.org:/home/git/ah_data /home/sd/Desktop/ah_data
# sudo killall -9 sshfs
# sudo fusermount -u /home/sd/Desktop/ah_data


data_root=/home/sd/Desktop/ah_data/2016-12-experiments-in-alden-hall
tmp=~/Desktop

desktop_file=$data_root/desktop/alden_hall_experiment_desktop_2016-12-28_19-04-03_EST-0500.mp4
desktop_start_time=10
cam_file="$data_root/video/2016-12-(20-29) - runs/HD4K/BDMV/STREAM/00047.MTS"
cam_start_time=910
cam_duration=120

outdir=$tmp/1

# desktop_file
# desktop_start_time
# cam_file
# cam_start_time


# #crop=WIDTH:HEIGHT:X_OFFSET:Y_OFFSET (top left corner)
# ffmpeg -i $desktop_video -filter:v "crop=541:777:86:92" -crf 18 $outdir/map.mp4
# ffmpeg -i $desktop_video -filter:v "crop=670:777:824:92" -crf 18 $outdir/div.mp4
# # # ffmpeg  -ss 00:15:10.000 -t 120 -i "${cam_file}" -filter:v "crop=1920:400:0:660" -crf 18 $outdir/cam.mp4

# $cam_video
# $outdir/cam.mp4
# trim=start=20:end=30,
# [2:v] trim=start=910:end=1030, setpts=PTS-STARTPTS, crop=1920:400:0:660 [cam];\
# combine videos https://trac.ffmpeg.org/wiki/Create%20a%20mosaic%20out%20of%20several%20input%20videos
# # # ffmpeg \
# # #     -i $desktop_file -i "$cam_file" \
# # #     -filter_complex " \
# # #         nullsrc=size=1920x1178 [base];  \
# # #         [0:v] trim=start=$desktop_start_time, setpts=PTS-STARTPTS,  crop=541:777:86:92  [map];\
# # #         [0:v] trim=start=$desktop_start_time, setpts=PTS-STARTPTS,  crop=670:777:824:92 [div];\
# # #         [1:v]  trim=start=$cam_start_time:end=1030,  setpts=PTS-STARTPTS, crop=1920:400:0:660 [vcam];\
# # #         [1:a] atrim=start=$cam_start_time:end=1030, asetpts=PTS-STARTPTS [acam];\
# # #         [base][map] overlay=shortest=1:x=0:y=0 [tmp1];\
# # #         [tmp1][div] overlay=shortest=1:x=550:y=0 [tmp2];\
# # #         [tmp2][vcam] overlay=shortest=1:x=0:y=777 [vout]\
# # #     "\
# # #     -c:v libx264 -map "[vout]" -map "[acam]" $outdir/output1111111.mp4

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
    -c:v libx264 $outdir/output.mp4



# ffmpeg \
#     -i "$cam_file" \
#     -vf " \
#         nullsrc=size=1920x1178 [base];  \
#         [0:v] trim=start=$cam_start_time:end=1030,trim=start=$cam_start_time:end=1030, setpts=PTS-STARTPTS, crop=1920:400:0:660 [cam];\
#         [base][cam] overlay=shortest=1:x=0:y=777\
#     "\
#     -c:v libx264 $outdir/output3.mp4

# ffmpeg \
#     -i "$data_root/video/2016-12-(20-29) - runs/HD4K/BDMV/STREAM/00069.MTS" \
#     -map 0:v\
#     -filter_complex " \
#         [0:v] trim=start=20:end=30, setpts=PTS-STARTPTS\
#     "\
#     -c:v libx264 $outdir/output8.mp4

# ffmpeg \
#     -i "$data_root/video/2016-12-(20-29) - runs/HD4K/BDMV/STREAM/00069.MTS" \
#     -filter_complex " \
#         [0:v] trim=start=20:end=30, setpts=PTS-STARTPTS  [vout];\
#         [0:a] atrim=start=20:end=30, asetpts=PTS-STARTPTS  [aout]\
#     "\
#     -map "[vout]" -map "[aout]" -c:v libx264 $outdir/output9.mp4