#!/usr/bin/env python

from Tkinter import Tk, Frame, Canvas, Text, BOTH, SW, S
import tkFont as font
import sys  
import os
import time
import rospy
usage = """
USAGE:
    ./make_legend.py   operator_name   width   window_x   window_y

"""

# canvas.create_text(0.1, 0.1, font=fontNorm, text="yellow")

if __name__ == "__main__":
    rospy.init_node('legend_display')
    key_mappings = rospy.get_param('/key_mappings',
                                   {"press button":     0,
                                    "don't press":    1})
    color_rgbs   = rospy.get_param('color_rgbs', ["#ffff00","#0000ff","#00ff00","#ff0000"])
    color_names  = rospy.get_param('color_names', ["yellow", "blue", "green", "red"])
    
    width = int(sys.argv[2])
    window_x = int(sys.argv[3])
    window_y = int(sys.argv[4])

    #color_rgbs = [(255, 255, 0), (0, 0, 255), (0, 255, 0), (255, 0, 0)]
    #color_names = ["yellow", "blue", "green", "red"]

    w = 0.01*width  # canvas.winfo_width()
    h = 0.01*200  # canvas.winfo_height()

    name_x = 5
    patch_x = 30
    expr_x = 60
    offset_y = 20
    patch_w = 20
    patch_h = 0.9*offset_y
    patch_offest_y = 5

    header_y = 15
    row1_y = header_y+20

    mappings = {v: k for k, v in key_mappings.items()}

    height = h*row1_y + offset_y*(2+len(mappings.keys()))

    geom = "%dx%d+%d+%d" % (width, height, window_x, window_y)

    root = Tk()
    root.geometry(geom)

    canvas = Canvas(root)
    canvas.pack(fill=BOTH, expand=1)

    fontBold = font.Font(family='Helvetica',
                         size=int(w*3.7), weight='bold')
    fontNorm = font.Font(family='Helvetica',
                         size=int(w*3.7), weight='normal')

    canvas.create_text(w*name_x,  h*header_y,
                        font=fontBold, anchor=SW, text="color")
    canvas.create_text(w*patch_x, h*header_y,
                        font=fontBold, anchor=SW, text="sample")
    canvas.create_text(w*expr_x,  h*header_y,
                        font=fontBold, anchor=SW, text="action")

    for k in sorted(mappings.keys()):
        row_y = h*(row1_y+k*offset_y)
        canvas.create_text(w*name_x,  row_y, font=fontNorm,
                            anchor=SW, text=color_names[k])
        canvas.create_rectangle(
            w*patch_x,
            row_y + h*patch_offest_y,
            w*(patch_x+patch_w),
            row_y-h*(patch_h-patch_offest_y),
            fill=color_rgbs[k])
        canvas.create_text(w*expr_x,  row_y, font=fontNorm,
                            anchor=SW, text=mappings[k])
    root.update()
    #root.mainloop()

    rospy.spin()

