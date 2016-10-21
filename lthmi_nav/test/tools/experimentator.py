#!/usr/bin/env python

cmd_template = ("roslaunch lthmi_nav run.launch  div:=%(div)s  pos:=%(pos)s  T:=%(T)f  vel:=%(vel)f  mx:=%(mx)s  popt:=%(popt)s  map:=%(map)s  resol:=%(resol)f  path:=%(path)d  tries:=%(tries)d  "
                                                 "prob_thresh_high:=%(prob_thresh_high)f  prob_thresh_low:=%(prob_thresh_low)f  prob_eps:=%(prob_eps)f  ksafe:=%(ksafe)f  rviz:=%(rviz)s")

expset = {
    'div': ['vtile', 'equidist'],      # <!-- map division policy:       vtile  htile  equidist  extremal -->
    'pos': ['maxprob_obst', 'cog2lopt'],       #   ra_maxprob  maxprob_euq  maxprob_obst  cog_euq  nearcog_euq  nearcog_obst  cog2lopt  cog2gopt -->
    'T': 1.0,
    'vel': 3.0,
    'mx': ['mx55', 'mx85'],
    'map': 'ak500inflated',
    'resol': 0.1,
    'path': [7, 12, 59],
    'tries': 30,
    'prob_thresh_high': 0.9,
    'prob_thresh_high': 0.9,
    'prob_eps': 1.0e-12,
    'ksafe': 0.9,
    'rviz': 'no'
}







#experimentator.py
    

#data
    #run-$date
        
        #data.csv
        #record.bag
    #database.txt