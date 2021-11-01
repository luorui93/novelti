#!/usr/bin/env python

import rosbag
import sys


doc="""
USAGE:
    ./rosbag_order_by_stamp.py input.bag output.bag
"""

if __name__=="__main__":
    if len(sys.argv)<3:
        sys.stderr.write(doc)
        exit(1)
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    with rosbag.Bag(output_file, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_file).read_messages():
            if topic == "/tf" and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)