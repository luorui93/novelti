#!/usr/bin/python
from __future__ import print_function
from data_process import DataBase
from collections import defaultdict
import bagplot
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import logging
import os
import pickle
import numpy as np
# import pandas as pd
import csv

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
cache_path ='/home/yaphes/humane_data/organized/database.pickle'
csv_path = '/home/yaphes/humane_data/organized/database.csv'
database = None

path_dict = {
    'a2b': 'A to B',
    'b2c': 'B to C',
    'c2a': 'C to A'
}

if __name__ == '__main__':
    if os.path.isfile(cache_path):
        with open(cache_path, 'rb') as cache:
            logger.info("Reading data from cache file %s" % cache_path)
            database = pickle.load(cache)
            #database.updateData()
            #
            # This is the list of all items stored in one data entry
            #['total_travel_distance', 'ang_dist', 'dist', 'ang_entropy', 'timestamp', 'user', 'goal_orientation_error', 
            # 'entropy', 'file_id', 'detected_cmd_number', 'goal_position_error', 'path', 'time_to_goal']
            #
            # print (database.queryByFileId("aykut-rob-novel-button-b2c-1-2019_02_28_18_58_50")['time_to_goal'])
    else:
        logger.info("No cache file found, start building a new one")

    with open(csv_path, 'w') as csvfile:
        fieldnames = ['Control Methods', 'Path', 'Arrival Position Error (m)', 'Arrival Orientation Error (rad)', 'Navigation Time (s)', 'Total Commands']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for control_id, data_group in database.data.items():
            for path, entry_list in data_group.items():
                for entry in entry_list:
                    row = {}
                    row['Control Methods'] = control_id.replace('-', '+')
                    row['Path'] = path_dict[path]
                    row['Arrival Position Error (m)'] = entry['goal_position_error']
                    row['Arrival Orientation Error (rad)'] = entry['goal_orientation_error']
                    row['Navigation Time (s)'] = entry['time_to_goal']
                    row['Total Commands'] = entry['detected_cmd_number']
                    writer.writerow(row)
                


    