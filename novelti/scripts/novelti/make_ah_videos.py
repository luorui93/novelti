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
            cwaveToolPath = "/home/sd/ws/devel/lib/novelti/cwave_cmdline",
            stamp = "2016-12-28_19-04-03_EST-0500",
            outFile = "/home/sd/Desktop/1/plots.mp4"):
    width_px = 708 #ideally 709, but odd number not supported by codec
    height_px = 777

    db = RecordByStamp(bagDir, cacheDir, cwaveToolPath)
    bagRecord = db.getRecord(stamp)
    
    fig1, axarr = plt.subplots(2, 1, sharex=True, facecolor='white')

    dpi = fig1.get_dpi()
    fig1.set_size_inches(float(width_px)/float(dpi),float(height_px)/float(dpi))
    t_end = max(bagRecord['entropy']['t'] + bagRecord['dist']['t'])
    
    entr_line = axarr[0].plot([], [], color="blue", linestyle='-', linewidth=2)[0]
    axarr[0].grid(True)
    axarr[0].autoscale(True)
    axarr[0].set_title("PDF entropy, bits", y=1.00, fontsize=16)
    #axarr[0].set_ylabel("PDF entropy, bits", fontsize=16)
    #axarr[0].set_xlabel("Time, sec", fontsize=16)
    axarr[0].set_xlim(min(bagRecord['entropy']['t']), t_end)
    axarr[0].set_ylim(min(bagRecord['entropy']['v']), 1.05*max(bagRecord['entropy']['v']))


    dist_line = axarr[1].plot([], [], color="blue", linestyle='-', linewidth=2)[0]
    axarr[1].grid(True)
    axarr[1].autoscale(True)
    axarr[1].set_title("Distance to goal, m", y=1.00, fontsize=16)
    #axarr[1].set_ylabel("Distance to goal, m", fontsize=16)
    axarr[1].set_xlabel("Time, sec", fontsize=16)
    axarr[1].set_xlim(min(bagRecord['dist']['t']), t_end)
    axarr[1].set_ylim(min(bagRecord['dist']['v']), 1.05*max(bagRecord['dist']['v']))

    period = 50

    frames = int(float(t_end)/(0.001*period))+2
    anim = animation.FuncAnimation(fig1, update_plots, frames, fargs=(period, bagRecord, entr_line, dist_line, axarr), interval=period, blit=True, repeat=False)

    anim.save(outFile, codec="libx264", dpi=dpi)


            
def combine_all_vids(ffmpegBin, title, desktopFile, camFile, bagVideoFile, outFile,  
                     desktopStartTime, camStartTime, camOffset, camOffY, camDuration, bagOffsetTime):
    # http://ffmpeg.org/ffmpeg-filters.html
    width   = 1920
    camH    = 400
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
            ' -itsoffset %d' %  camOffset + \
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
            " -metadata title=\"Robotic wheelchair controlled via a low-throughput human-machine interface. Experiment: %s\"" % title + \
            " -metadata year=\"2017\"" + \
            " -metadata author=\"Dmitry A. Sinyukov\"" + \
            " -metadata artist=\"Dmitry A. Sinyukov\"" + \
            " -c:v   libx264 " + \
            outFile
    print command
    output,error  = subprocess.Popen(
                    command, shell=True, universal_newlines=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
    #print output
    #print error

#plt.show()

def getDesktopVidByStartStamp(dataRoot, startStamp):
    startTime = datetime.strptime(startStamp, '%Y-%m-%d_%H-%M-%S_EST-0500')
    desktopTime = None
    desktopFile = None
    for fname in os.listdir(dataRoot+"/desktop"):
        if fname.startswith("alden_hall_experiment_desktop_"):
            t = datetime.strptime(fname, 'alden_hall_experiment_desktop_%Y-%m-%d_%H-%M-%S_EST-0500.mp4')
            if t>=startTime and (desktopTime is None or t<desktopTime):
                desktopTime = t
                desktopFile = fname
    return dataRoot+"/desktop/"+desktopFile



class CamVideoBase:
    def __init__(self, dataRoot, ffprobePath="/usr/bin/ffprobe"):
        self.root = dataRoot
        self.ffprobePath = ffprobePath
        
        pklFilePath = os.path.join(dataRoot, "video", "camVideoMetaData.pkl")
        if os.path.isfile(pklFilePath):
            info("Reading cam video file meta data from cache file %s" % pklFilePath)
            with open(pklFilePath, 'rb') as input1:
                self.metas = pickle.load(input1)
        else:
            self.readMetas()
            info("Saving cam video file meta data to cache as %s" % pklFilePath)
            with open(pklFilePath, 'wb') as output:
                pickle.dump(self.metas, output, pickle.HIGHEST_PROTOCOL)
    
    def readMetas(self):
        self.metas = []
        for path, dirs, files in os.walk(os.path.join(self.root, "video")):
            #path = root.split(os.sep)
            for file1 in files:
                if file1.endswith(".MTS"):
                    fullPath = os.path.join(path, file1)
                    relPath =  os.path.relpath(fullPath, self.root)
                    getVideoDurationCmd = '%s -v error -show_entries format=duration   -of default=noprint_wrappers=1:nokey=1 "%s"' % (self.ffprobePath, fullPath)
                    duration = float(subprocess.check_output(getVideoDurationCmd, shell=True))
                    endTime = os.path.getmtime(fullPath)
                    
                    if relPath.find("2016-12-31")>=0: # this is a correction for the problem with the modification time of cam file recorded on that day 
                        #search "TIME DISCREPANCY" in the alden-hall-protocol.txt
                        if relPath.find("HD4K")>=0: 
                            file_num = int(os.path.splitext(os.path.basename(file1))[0])
                            if file_num<=11:
                                endTime = 1483232388.0 - 30355  + file_num*(17*60+2.5)#(modif time of 00000.MTS) - (correction calculation in TIME DISCREPANCY) + (file num)*(duration of video)
                            elif file_num==12:
                                endTime = 1483232388.0 - 30355  + 11*(17*60+2.5) + (8*60+34)
                            elif file_num<=20:
                                endTime = 1483222946.0 + 15 + (17*60+2- (16*60+15))+ (file_num-15)*(17*60+2.5) #...(start of desktop video) + (Got new plan moment)
                            else:
                                endTime = 1483222946.0 + 15 + (17*60+2- (16*60+15))+ (20-15)*(17*60+2.5)   + 12*60+39#
                    info = (endTime, relPath, duration)
                    print info
                    self.metas.append(info)
        self.metas = sorted(self.metas, key=lambda x: x[0])

    def get4KVidByStartStamp(self, startStamp):
        #print startStamp
        epoch = datetime.utcfromtimestamp(0)
        startTime = (datetime.strptime(startStamp, '%Y-%m-%d_%H-%M-%S_EST-0500') - epoch).total_seconds()+3600*5
        camTime = None
        for (endTime, relPath, duration) in self.metas:
            if relPath.find("HD4K")>=0 and endTime>=startTime:
                fullPath = os.path.join(self.root, relPath)
                timeStart = startTime-(endTime-duration)
                return (fullPath, timeStart)
        
        
videos = [
    # name              title                           desktop video stamp             desktop t_start     cam t_start     cam t_offset    bag t_offset    cam dur8n camOffY
    ("non-pois",        "nav to non-POIs",              "2016-12-31_18-46-21_EST-0500", 5,                  4.7,            0.0,            4.2,            180,     660), #cam<13.7 bag<=4.3
    ("goal-mark",       "Goal marker",                  "2017-01-01_18-36-44_EST-0500", 5,                  12.1,           0.0,            6.65,           55,      610), #cam<12.4   6.4<6.55<bag<6.7
    ("pois",            "Effect of POIs",               "2016-12-29_21-55-19_EST-0500", 5,                  6.8,            0.0,            3.7,            55,      610), #1.8<bag<2.8?... bag>=3.6
    ("smooth",          "Smoothening",                  "2017-01-01_21-17-06_EST-0500", 5,                  13.1,           0.0,            4.9,            90,      610), #cam>12.8  4.7<bag<5.0
    ("change-intent",   "Change of destination",        "2016-12-31_00-02-24_EST-0500", 18.5,               21.7,           0.0,            3.1,            135,     660),
    ("simple",          "Simple experiment",            "2016-12-28_19-04-03_EST-0500", 10,                 8.7,            0.0,            2.5,            87,      660),
    ("div-extredist",   "extredist map segmentation",   "2017-01-02_12-17-58_EST-0500", 5,                  12.4,           0.0,            4.2,            100,     610), #4.0<bag<7.0
    ("div-altertile",   "altertile map segmentation",   "2017-01-02_10-47-38_EST-0500", 5,                  11.9,           0.0,            8.4,            60,      610), #cam<12.4, bag>7.7
    ("div-nearcog_extr","nearcog_extr map segmentation","2017-01-02_09-15-12_EST-0500", 5,                  12.0,           0.0,            9.9,            77,      610), #cam<12.4, bag<7.0? bag>9.7>5.7
    ("pos-no-move",     "no_move pose selection",       "2017-01-02_11-35-47_EST-0500", 5,                  14.0,           0.0,            10.7,           100,     610), #cam>12.4, bag>10
    ("pos-cog2lopt",    "cog2lopt pose selection",      "2017-01-02_12-07-39_EST-0500", 23,                 30.8,           0.0,            3.7,            65,      610), #cam>=30.4, bag<=4.0
    ("pos-nearcog-obst","nearcog_obst pose selection",  "2017-01-02_12-38-16_EST-0500", 5,                  12.4,           0.0,            5.5,            60,      610), #cam, bag<7
]

if __name__=="__main__":
    # sudo sshfs -o allow_other,reconnect -p 2222 git@zzzzzz.no-ip.org:/home/git/ah_data /home/sd/Desktop/ah_data
    # sudo killall -9 sshfs
    # sudo fusermount -u /home/sd/Desktop/ah_data

    ffmpegBin       = "/usr/bin/ffmpeg", 
    dataRoot        = "/home/sd/Desktop/ah_data/2016-12-experiments-in-alden-hall"
    outDir          = "/home/sd/Desktop/1"
    cacheDir        = "/home/sd/Desktop/ah_data_cache"
    cwaveToolPath   = "/home/sd/ws/devel/lib/novelti/cwave_cmdline"
    
    camBase = CamVideoBase(dataRoot)
    
    for vidPrms in videos:
        (name, title, stamp, desktopStartTime, camStartTime, camOffset, bagOffsetTime, camDuration, camOffY) = vidPrms
        print "======== Making a combo video '%s' stamp=%s =========" % (title,stamp)
        bagVideoFile = outDir+"/bag-only-"+stamp+".mp4"
        if not os.path.isfile(bagVideoFile):
            print "Making a video from bag file"
            bag2plot(dataRoot + "/bags", cacheDir, cwaveToolPath, stamp, bagVideoFile)
        (camFile, camTimeStartBase) = camBase.get4KVidByStartStamp(stamp)
        #print "(camFile, camTimeStartBase) = %s" % str((camFile, camTimeStartBase))
        desktopFile = getDesktopVidByStartStamp(dataRoot, stamp)
        combine_all_vids(ffmpegBin, 
                        title,
                        desktopFile,
                        camFile, #camFile
                        bagVideoFile, 
                        outDir +"/combo--"+name+"--"+stamp+".mp4",  #outFile
                        desktopStartTime, camTimeStartBase+camStartTime, camOffset, camOffY, camDuration, bagOffsetTime)
