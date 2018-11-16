#!/usr/bin/python3 
import yaml #sudo apt-get install python3-yam
import sys
import os
import time
usage = """
USAGE:
    ./make_legend.py   operator_name   width   window_x   window_y

"""

datadir = os.environ['HOME']+"/humane_data"


from tkinter import Tk,Frame, font, Canvas, Text, BOTH, SW, S



#canvas.create_text(0.1, 0.1, font=fontNorm, text="yellow")



if len(sys.argv)!=5:
    print(usage)
    exit(1)

width = int(sys.argv[2])
window_x = int(sys.argv[3])
window_y = int(sys.argv[4])

color_rgbs  = [(255,255,0), (0,0,255), (0,255,0), (255,0,0)]
color_names = ["yellow", "blue", "green", "red"]
expressions = ["clench", "smirk left", "smirk right", "raise eyebrow"]



w = 0.01*width #canvas.winfo_width()
h = 0.01*200 #canvas.winfo_height()

name_x = 5
patch_x = 30
expr_x = 60
offset_y = 20
patch_w = 20
patch_h = 0.9*offset_y
patch_offest_y = 5

header_y = 15
row1_y = header_y+20



with open(datadir + "/emotiv_cmd_mapping/" + sys.argv[1]+ "_mapping.yaml") as f:
    mappings = {v: k for k, v in yaml.load(f)["key_mappings"].items()}
    
    height = h*row1_y+ offset_y*(2+len(mappings.keys()))

    geom="%dx%d+%d+%d" % (width, height, window_x, window_y)

    root = Tk()
    root.geometry(geom) 

    canvas = Canvas(root)
    canvas.pack(fill=BOTH, expand=1)
    
    fontBold = font.Font(family='Helvetica', size=int(w*3.7), weight='bold')
    fontNorm = font.Font(family='Helvetica', size=int(w*3.7), weight='normal')
    
    canvas.create_text(w*name_x,  h*header_y, font=fontBold, anchor=SW, text="color")
    canvas.create_text(w*patch_x, h*header_y, font=fontBold, anchor=SW, text="sample")
    canvas.create_text(w*expr_x,  h*header_y, font=fontBold, anchor=SW, text="expression")
    
    for k in sorted(mappings.keys()):
        row_y = h*(row1_y+k*offset_y)
        canvas.create_text(w*name_x,  row_y, font=fontNorm, anchor=SW, text=color_names[k])
        color = '#%02x%02x%02x' % color_rgbs[k]
        canvas.create_rectangle(
            w*patch_x, 
            row_y + h*patch_offest_y, 
            w*(patch_x+patch_w), 
            row_y-h*(patch_h-patch_offest_y), 
            fill=color)
        canvas.create_text(w*expr_x,  row_y, font=fontNorm, anchor=SW, text=mappings[k])
    root.update()
    root.mainloop()
    while root.is_alive():
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            root.destroy()
            break
    #draw header:
        #color name color expression
    
    """
        text_file.write(tmpl1 % {"height" : 110 + dy*len(color_rgbs) })
        for k in sorted(mappings.keys()):
            text_file.write(tmpl2 % {
                "y":    110+k*dy, 
                "recy": 70+k*dy, 
                "r":    color_rgbs[k][0], 
                "g":    color_rgbs[k][1], 
                "b":    color_rgbs[k][2],
                "expr": mappings[k],
                "name": color_names[k]
            })
        text_file.write(tmpl3)
    """