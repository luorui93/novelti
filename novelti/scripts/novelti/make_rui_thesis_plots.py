#!/usr/bin/env python
import os
import sys


from Bag2Plot import *
import matplotlib.gridspec as gridspec
import matplotlib.lines as mlines
from subprocess import call

DBASE = None;

LOCATIONS = {
    #name          x       y       yaw
    "door1":      (11.78,  1.85,   1.57079),
    "livroom1":   (8.7,    13.9,   1.57079),
    "office1":    (25.6,   18.0,   0.0),
    "bathroom1":  (3.4,    1.9,    1.57079),
    "music1":     (13.8,   23.4,   0.0),
    "bedroom1":   (44.8,   6.4,   3.14159),
    "door2":      (9.0,    3.4,    1.57079),
    "kitchen1":   (18.8,   29.6,   -1.57079),
    "livroom2":   (7.71,   17.15,  -1.57079),
    "storage1":   (13.01,  9.71,   1.57079),
    "storage2":   (15.26,  10.26,   0.0),
    "storage3":   (15.27,  8.66,   -1.57079),
    "pos1":       (33.5,   28.7,   0)
}

BAG_STAMPS = {
    "mx100_no_pois": {
        "kitchen1tobedroom1":    ['_2017-12-03-22-15-00','_2017-12-03-22-17-05','_2017-12-03-22-19-47'],
        "livroom1tooffice1":     ['_2017-12-04-09-08-24','_2017-12-04-09-10-27','_2017-12-04-09-11-46']
    },
    "mx85_no_pois": {
        "kitchen1tobedroom1":    ['_2017-12-03-15-06-22','_2017-12-03-22-05-27','_2017-12-03-22-07-36'], 
        "livroom1tooffice1":     ['_2017-12-04-09-05-45','_2017-12-04-09-23-51','_2017-12-04-09-25-49']
    }, 
    "mx91_no_pois": {
        "kitchen1tobedroom1":     ['_2017-12-03-22-23-41','_2017-12-03-22-26-15','_2017-12-03-22-28-24'],
        "livroom1tooffice1":      ['_2017-12-04-09-15-49','_2017-12-04-09-18-34','_2017-12-04-09-19-57']
    },
    "mx70_no_pois": {
        "kitchen1tobedroom1":     ['_2017-12-03-22-31-55','_2017-12-03-22-34-57','_2017-12-03-22-38-58'],
        "livroom1tooffice1" :     ['_2017-12-04-09-28-31','_2017-12-04-09-33-47','_2017-12-04-09-37-18']
    },
    "mx100_change_mind":{
        "livroom1tooffice1": ["_2017-12-04-10-18-43"]
    },
    "mx91_change_mind": {
        "livroom1tooffice1": ["_2017-12-04-10-23-19"]
    },
    "mx85_change_mind": {
        "livroom1tooffice1": ["_2017-12-04-10-25-55"]
    },
    "human_subjects": {
        "livroom1topos1":    ["_2017-12-04-11-11-29"]
    }
}

def plot_set_of_tries(ax_map, ax_dist, ax_entr, bag_stamps, colors):
    lines = []
    for k,stamp in enumerate(bag_stamps):
        bagRecord = DBASE.getRecord(stamp)
        pbr = PlottableBagRecord(bagRecord)
        lines.append(pbr.plotPathDistEntropy(ax_map, ax_dist, ax_entr, colors[k])[0])
    return lines

def plot_on_1map(fig, sub_grid, locations, method_names, method_ids, method_color_sets, route_names, route_ids, stamp_sets):
    lines = []
    num_routes = len(route_names)
    gs = gridspec.GridSpecFromSubplotSpec(2, num_routes+1, 
            subplot_spec=sub_grid,
            hspace=0.1) #left=0.02, right=0.98, bottom=0.04, top=0.96) #, wspace=0.05)
    ax_map = plt.Subplot(fig, gs[:,0])
    fig.add_subplot(ax_map)
    map_stamp = stamp_sets[method_ids[0]][route_ids[0]][0]
    PlottableBagRecord(DBASE.getRecord(map_stamp)).plotMapInflated(ax_map)
    #draw labels for locations:
    for k, loc in enumerate(locations):
        ax_map.annotate(loc,
            xy=LOCATIONS[loc][:2], 
            xycoords='data',
            xytext=(0, 25), 
            textcoords='offset points',
            backgroundcolor=(1.0, 1.0, 0.0, 0.7),
            #linewidth=0.1,
            arrowprops=dict(
                facecolor=(1.0, 1.0, 0.0, 0.7), 
                linewidth=0.1,
                #edgecolor='none',
                #alpha=0.7, 
                shrink=0.05, 
                frac=0.5, 
                width=4, 
                headwidth=9 ), #http://matplotlib.org/users/annotations_intro.html
            horizontalalignment='center', 
            verticalalignment='bottom', 
            fontsize=16)
    for route_k,route_name in enumerate(route_names):
        route_id  = route_ids[route_k]
        ax_entr = plt.Subplot(fig, gs[0,route_k+1])
        ax_entr.set_title(route_names[route_k], fontsize=20)
        fig.add_subplot(ax_entr)
        ax_dist = plt.Subplot(fig, gs[1,route_k+1], sharex=ax_entr)
        fig.add_subplot(ax_dist)
        for method_k, method in enumerate(method_names):
            method_id = method_ids[method_k]
            stamp_list = stamp_sets[method_id][route_id]
            ls = plot_set_of_tries(ax_map, ax_dist, ax_entr, stamp_list, method_color_sets[method_k])
            if route_k==0:
                lines += ls
    return lines

def plot_compare_4route_2map(title, method_names, method_ids, method_color_sets):
    fig = plt.figure(figsize=(2*8.5, 2*11.0), facecolor='white')
    #plt.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.1)
    outer_grid = gridspec.GridSpec(2, 1)
    outer_grid.update(left=0.02, right=0.98, bottom=0.04, top=0.9) #, wspace=0.05)
    lines = plot_on_1map(fig, outer_grid[0], 
                locations= ["kitchen1","bedroom1"],
                method_names=      method_names,
                method_ids=        method_ids,
                method_color_sets= method_color_sets,
                route_names= ["kitchen1->bedroom1"],
                route_ids=   ["kitchen1tobedroom1",],
                stamp_sets=BAG_STAMPS)
    plot_on_1map(fig, outer_grid[1], 
                locations= ["livroom1","office1"],
                method_names=      method_names,
                method_ids=        method_ids,
                method_color_sets= method_color_sets,
                route_names= ["livroom1->office1"],
                route_ids=   ["livroom1tooffice1"],
                stamp_sets=BAG_STAMPS)
    #plt.tight_layout()
    fig.autolayout = True
    
    ntries = 3
    labels = []
    handles=[]
    for try_k in xrange(ntries):
        for method_k,method_name in enumerate(method_names):
            handle = mlines.Line2D([], [], 
                color=method_color_sets[method_k][try_k],
                linewidth=2) #, label="%s, try %d"%(method_name,try_k))
            handles.append(handle)
            labels.append("%s, try %d"%(method_name,try_k+1))
    fig.legend( handles, labels, loc='upper left', bbox_to_anchor=[0.02,0.0,0.98,0.95], ncol=ntries )
    fig.suptitle(title, fontsize=25,horizontalalignment='center')
    return outer_grid

def plot_change_mind(title, method_names, method_ids, method_color_sets):
    fig = plt.figure(figsize=(2*8.5, 2*11.0), facecolor='white')
    #plt.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.1)
    outer_grid = gridspec.GridSpec(3, 1)
    outer_grid.update(left=0.02, right=0.98, bottom=0.04, top=0.9) #, wspace=0.05)
    lines =plot_on_1map(fig, outer_grid[0], 
                locations= ["livroom1","office1"],
                method_names=      method_names,
                method_ids=        method_ids,
                method_color_sets= method_color_sets,
                route_names= ["livroom1->office1"],
                route_ids=   ["livroom1tooffice1"],
                stamp_sets=BAG_STAMPS)
    # plot_on_1map(fig, outer_grid[1], 
    #             locations= ["livroom1","office1"],
    #             method_names=      method_names,
    #             method_ids=        method_ids,
    #             method_color_sets= method_color_sets,
    #             route_names= ["livroom1->office1"],
    #             route_ids=   ["livroom1tooffice1"],
    #             stamp_sets=BAG_STAMPS)
    # plot_on_1map(fig, outer_grid[2], 
    #             locations= ["livroom1","office1"],
    #             method_names=      method_names,
    #             method_ids=        method_ids,
    #             method_color_sets= method_color_sets,
    #             route_names= ["livroom1->office1"],
    #             route_ids=   ["livroom1tooffice1"],
    #             stamp_sets=BAG_STAMPS)

    #plt.tight_layout()
    fig.autolayout = True
    
    ntries = 1
    labels = []
    handles=[]
    for try_k in xrange(ntries):
        for method_k,method_name in enumerate(method_names):
            handle = mlines.Line2D([], [], 
                color=method_color_sets[method_k][try_k],
                linewidth=2) #, label="%s, try %d"%(method_name,try_k))
            handles.append(handle)
            labels.append("%s, try %d"%(method_name,try_k+1))
    fig.legend( handles, labels, loc='upper left', bbox_to_anchor=[0.02,0.0,0.98,0.95], ncol=ntries )
    fig.suptitle(title, fontsize=25,horizontalalignment='center')
    return outer_grid

def plot_human_exp(title, method_names, method_ids, method_color_sets):
    fig = plt.figure(figsize=(2*8.5, 2*11.0), facecolor='white')
    #plt.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.1)
    outer_grid = gridspec.GridSpec(3, 1)
    outer_grid.update(left=0.02, right=0.98, bottom=0.04, top=0.9) #, wspace=0.05)
    lines =plot_on_1map(fig, outer_grid[0], 
                locations= ["livroom1","pos1"],
                method_names=      method_names,
                method_ids=        method_ids,
                method_color_sets= method_color_sets,
                route_names= ["livroom1->pos1"],
                route_ids=   ["livroom1topos1"],
                stamp_sets=BAG_STAMPS)
    plot_on_1map(fig, outer_grid[1], 
                locations= ["livroom1","pos1"],
                method_names=      method_names,
                method_ids=        method_ids,
                method_color_sets= method_color_sets,
                route_names= ["livroom1->pos1"],
                route_ids=   ["livroom1topos1"],
                stamp_sets=BAG_STAMPS)
    # plot_on_1map(fig, outer_grid[2], 
    #             locations= ["livroom1","office1"],
    #             method_names=      method_names,
    #             method_ids=        method_ids,
    #             method_color_sets= method_color_sets,
    #             route_names= ["livroom1->office1"],
    #             route_ids=   ["livroom1tooffice1"],
    #             stamp_sets=BAG_STAMPS)

    #plt.tight_layout()
    fig.autolayout = True
    
    ntries = 1
    labels = []
    handles=[]
    for try_k in xrange(ntries):
        for method_k,method_name in enumerate(method_names):
            handle = mlines.Line2D([], [], 
                color=method_color_sets[method_k][try_k],
                linewidth=2) #, label="%s, try %d"%(method_name,try_k))
            handles.append(handle)
            labels.append("%s, try %d"%(method_name,try_k+1))
    fig.legend( handles, labels, loc='upper left', bbox_to_anchor=[0.02,0.0,0.98,0.95], ncol=ntries )
    fig.suptitle(title, fontsize=25,horizontalalignment='center')
    return outer_grid

def plot_human_exp_port(title, method_names, method_ids, out_file):
    print "\n\n======================================================================="
    print title
    print "======================================================================="
    blue_set = ["#9999FF","#3333FF","#000099"]
    red_set  = ["#FF9999","#FF3333","#BB0000"]
    
    plot_human_exp(
        title=title,
        method_names=      method_names,
        method_ids=        method_ids,
        method_color_sets= [blue_set,     red_set]
    )
    plt.savefig(out_file)
    #call(["evince", out_file])
    #plt.show()

def plot_compare2_4route_2map(title, method_names, method_ids, out_file):
    print "\n\n======================================================================="
    print title
    print "======================================================================="
    blue_set = ["#9999FF","#3333FF","#000099"]
    red_set  = ["#FF9999","#FF3333","#BB0000"]
    
    plot_compare_4route_2map(
        title=title,
        method_names=      method_names,
        method_ids=        method_ids,
        method_color_sets= [blue_set,     red_set]
    )
    plt.savefig(out_file)
    #call(["evince", out_file])
    #plt.show()

def plot_change_mind_port(title, method_names, method_ids, out_file):
    print "\n\n======================================================================="
    print title
    print "======================================================================="
    blue_set = ["#9999FF"]
    red_set = ["#FF9999"]
    green_set = ["#99FF99"]

    color_set = [blue_set, red_set, green_set]
    
    plot_change_mind(
        title=title,
        method_names=      method_names,
        method_ids=        method_ids,
        method_color_sets= color_set
    )
    plt.savefig(out_file)
    #call(["evince", out_file])
    #plt.show()    

def plots_for_thesis(output_file_prefix, plot_name):
    extension = ".pdf"
    if plot_name == "mx100-vs-mx91-no-pois":
        plot_compare2_4route_2map(
            title=          "mx100-vs-mx91-no-pois 2 routes",
            method_names=   ["100% HMI","91% HMI"],
            method_ids=     ["mx100_no_pois","mx91_no_pois"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "mx85-vs-mx70-no-pois":
        plot_compare2_4route_2map(
            title=          "mx85-vs-mx70-no-pois 2 routes",
            method_names=   ["85% HMI","70% HMI"],
            method_ids=     ["mx85_no_pois","mx70_no_pois"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "change-mind-different-mx":
        plot_change_mind_port(
            title=          "change-mind-different-mx",
            method_names=   ["100% HMI","91% HMI","85% HMI"],
            method_ids=     ["mx100_change_mind","mx91_change_mind","mx85_change_mind"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "human-exp":
        plot_human_exp_port(
            title=          "human experiments with 85% HMI",
            method_names=   ["85% HMI"],
            method_ids=     ["human_subjects"],
            out_file=       output_file_prefix + plot_name + extension
        )
    else:
        return False

    return True

if __name__=="__main__":
    bagDir = "/home/yaphes/bag/"
    cacheDir = "/home/yaphes/data_cache"
    cwaveToolPath = "/home/yaphes/novelti_ws/devel/lib/novelti/cwave_cmdline"
    outputDir = "/home/yaphes/plot"
    outputFile ="rui-thesis-human-exp.pdf"
  
    DBASE = RecordByStamp(bagDir, cacheDir, cwaveToolPath)
    
    thesis_prefix = "rui-thesis-"
    if outputFile.startswith(thesis_prefix):
        plot_name = outputFile[len(thesis_prefix):-4]
        if not plots_for_thesis(outputDir+"/"+thesis_prefix,  plot_name):
            print "Unknown plot name for thesis plot: '%s'" % plot_name
            exit(1)