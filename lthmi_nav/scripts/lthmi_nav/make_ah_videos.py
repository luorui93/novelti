#!/usr/bin/env python
import os
import sys


from Bag2Plot import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import subprocess


def update_plots(frame_num, period, bagRecord, entr_line, dist_line, axarr):
    cur_time = 0.001*period*frame_num
    #print frame_num
    #print cur_time
    for ind, t in enumerate(bagRecord['entropy']['t']):
        if t <= cur_time:
            entr_index = ind
        else:
            break
    entr_line.set_xdata(bagRecord['entropy']['t'][:entr_index+1])
    entr_line.set_ydata(bagRecord['entropy']['v'][:entr_index+1])

    for ind, t in enumerate(bagRecord['dist']['t']):
        if t <= cur_time:
            dist_index = ind
        else:
            break
    dist_line.set_xdata(bagRecord['dist']['t'][:dist_index+1])
    dist_line.set_ydata(bagRecord['dist']['v'][:dist_index+1])
    #axarr[0].relim()
    #axarr[0].autoscale_view(True,True,True)
    #axarr[1].relim()
    #axarr[1].autoscale_view(True,True,True)
    return [entr_line, dist_line]



def bag2plot(bagDir = "/home/sd/Desktop/ah_data/2016-12-experiments-in-alden-hall/bags",
            cacheDir = "/home/sd/Desktop/ah_data_cache",
            cwaveToolPath = "/home/sd/ws/devel/lib/lthmi_nav/cwave_cmdline",
            stamp = "2016-12-28_19-04-03_EST-0500",
            outFile = "/home/sd/Desktop/1/plots.mp4"):
    width_px = 708 #ideally 709, but odd number not supported by codec
    height_px = 777

    db = RecordByStamp(bagDir, cacheDir, cwaveToolPath)
    bagRecord = db.getRecord(stamp)
    
    fig1, axarr = plt.subplots(2, 1, sharex=True, facecolor='white')

    dpi = fig1.get_dpi()
    fig1.set_size_inches(float(width_px)/float(dpi),float(height_px)/float(dpi))

    entr_line = axarr[0].plot([], [], color="blue", linestyle='-', linewidth=2)[0]
    axarr[0].grid(True)
    axarr[0].autoscale(True)
    axarr[0].set_title("PDF entropy, bits", y=1.00, fontsize=16)
    #axarr[0].set_ylabel("PDF entropy, bits", fontsize=16)
    #axarr[0].set_xlabel("Time, sec", fontsize=16)
    axarr[0].set_xlim(min(bagRecord['entropy']['t']), max(bagRecord['entropy']['t']))
    axarr[0].set_ylim(min(bagRecord['entropy']['v']), 1.05*max(bagRecord['entropy']['v']))


    dist_line = axarr[1].plot([], [], color="blue", linestyle='-', linewidth=2)[0]
    axarr[1].grid(True)
    axarr[1].autoscale(True)
    axarr[1].set_title("Distance to goal, m", y=1.00, fontsize=16)
    #axarr[1].set_ylabel("Distance to goal, m", fontsize=16)
    axarr[1].set_xlabel("Time, sec", fontsize=16)
    axarr[1].set_xlim(min(bagRecord['dist']['t']), max(bagRecord['dist']['t']))
    axarr[1].set_ylim(min(bagRecord['dist']['v']), 1.05*max(bagRecord['dist']['v']))

    period = 50

    t_end = max(bagRecord['entropy']['t'] + bagRecord['dist']['t'])
    frames = int(float(t_end)/(0.001*period))+2
    anim = animation.FuncAnimation(fig1, update_plots, frames, fargs=(period, bagRecord, entr_line, dist_line, axarr), interval=period, blit=True, repeat=False)

    anim.save(outFile, codec="libx264", dpi=dpi)
    


#def combine_all_vids(ffmpeg_bin, desktop_file, cam_file, bag_video_file,   
                     #desktop_start_time, cam_start_time, cam_duration):
    #command = [ ffmpeg_bin,
            #'-i',   desktop_file,
            #'-ss',  str(cam_start_time),
            #'-t',   str(cam_duration),
            #'-i',   cam_file,
            #'-i',   bag_video_file,
            #'-filter_complex', '"nullsrc=size=1920x1178 [base]; [0:v] trim=start=%d, setpts=PTS-STARTPTS, split [map][div]; [map] crop=541:777:86:92  [vmap]; [div] crop=670:777:824:92 [vdiv]; [1:v] crop=1920:400:0:660 [vcam]; [base][vmap] overlay=shortest=1:x=0:y=0 [tmp1]; [tmp1][vdiv] overlay=shortest=1:x=541:y=0 [tmp2]; [tmp2][2:v]  overlay=shortest=1:x=1212:y=0[tmp3]; [tmp3][vcam] overlay=shortest=1:x=0:y=777"' % desktop_start_time,
            #'-metadata', 'title="Robotic wheelchair controlled via a low-throughput human-machine interface"',
            #'-metadata', 'year="2017"',
            #'-metadata', 'author="Dmitry A. Sinyukov"',
            #'-metadata', 'artist="Dmitry A. Sinyukov"',
            #'-c:v',  'libx264',
            #out_file]
            
            
def combine_all_vids(ffmpegBin, desktopFile, camFile, bagVideoFile, outFile,  
                     desktopStartTime, camStartTime, camDuration, bagOffsetTime):
    # http://ffmpeg.org/ffmpeg-filters.html
    width   = 1920
    camH    = 400
    camOffY = 660
    mapW    = 541
    mapH    = 777
    mapOffX = 86
    mapOffY = 92
    
    divW    = 670
    divOffX = 824
    
    height  = camH+mapH
    height += height%2
    command = "%s" % ffmpegBin + \
            ' -y' + \
            ' -i   "%s"' % desktopFile + \
            " -ss  %d" % camStartTime + \
            " -t   %d" % camDuration + \
            ' -i   "%s"' % camFile + \
            ' -itsoffset %d' %  bagOffsetTime + \
            ' -i   "%s"' % bagVideoFile + \
            " -filter_complex \"" + \
                    "nullsrc=size=%dx%d [base]; " % (width,height) +\
                    "[0:v] trim=start=%d, setpts=PTS-STARTPTS, split [map][div];" % desktopStartTime + \
                    "[map] crop=%d:%d:%d:%d  [vmap];" % (mapW,mapH,mapOffX,mapOffY) + \
                    "[div] crop=%d:%d:%d:%d [vdiv];"  % (divW,mapH,divOffX,mapOffY) + \
                    "[1:v] crop=%d:%d:0:%d [vcam];" % (width, camH, camOffY)+ \
                    "[base][vmap] overlay=shortest=1:x=0:y=0    [tmp1];"   + \
                    "[tmp1][vdiv] overlay=shortest=1:x=%d:y=0  [tmp2];" % mapW + \
                    "[tmp2][2:v]  overlay=x=%d:y=0 [tmp3];" % (mapW+divW+1) + \
                    "[tmp3][vcam] overlay=x=0:y=%d" % mapH  + \
            "\"" +\
            " -metadata title=\"Robotic wheelchair controlled via a low-throughput human-machine interface\"" + \
            " -metadata year=\"2017\"" + \
            " -metadata author=\"Dmitry A. Sinyukov\"" + \
            " -metadata artist=\"Dmitry A. Sinyukov\"" + \
            " -c:v   libx264 " + \
            outFile
    print command
    output,error  = subprocess.Popen(
                    command, shell=True, universal_newlines=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
    print output
    print error

#plt.show()



if __name__=="__main__":
    #if len(sys.argv)>5:
        #bag2plot(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])

    # sudo sshfs -o allow_other,reconnect -p 2222 git@zzzzzz.no-ip.org:/home/git/ah_data /home/sd/Desktop/ah_data
    # sudo killall -9 sshfs
    # sudo fusermount -u /home/sd/Desktop/ah_data

    ffmpegBin       = "/usr/bin/ffmpeg", 
    dataRoot        = "/home/sd/Desktop/ah_data/2016-12-experiments-in-alden-hall"
    outDir          = "/home/sd/Desktop/1"
    desktopFile     = dataRoot + "/desktop/alden_hall_experiment_desktop_2016-12-28_19-04-03_EST-0500.mp4"
    camFile         = dataRoot + "/video/2016-12-(20-29) - runs/HD4K/BDMV/STREAM/00047.MTS"
    desktopStartTime = 10
    camStartTime    = 909.2
    camDuration     = 80
    bagOffsetTime   = 2.5
    bagVideoFile    = outDir + "/plots.mp4"
    outFile         = outDir + "/output.mp4"
    bagDir          = dataRoot + "/bags"
    cacheDir        = "/home/sd/Desktop/ah_data_cache"
    cwaveToolPath   = "/home/sd/ws/devel/lib/lthmi_nav/cwave_cmdline"
    bagStamp        = "2016-12-28_19-04-03_EST-0500"
    #bag2plot(bagDir, cacheDir, cwaveToolPath, bagStamp, bagVideoFile)
        
    combine_all_vids(ffmpegBin, desktopFile, camFile, bagVideoFile, outFile,  
                     desktopStartTime, camStartTime, camDuration, bagOffsetTime)
    