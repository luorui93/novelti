#! /usr/bin/python
import rosbag
import sys
import ast
import os
import pickle
import logging
import time
import csv
from math import sin,cos,log,pi,sqrt
import numpy as np
from datetime import datetime
import StringIO
import tf_conversions
from subprocess import Popen, PIPE, STDOUT
from exceptions import RuntimeError

from MapTools import GridMap

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logging.basicConfig(format='%(levelname)s:%(message)s')
cwavetool_path = '/home/yaphes/anna_ws/devel/lib/novelti/cwave_cmdline'
data_dir = '/home/yaphes/humane_data/organized'
map_inflated_string = ""
map_inflated_resolution = 0.0

def timer(method):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = method(*args, **kwargs)
        elapsed_time = round((time.time() - start_time), 3)
        logger.debug("[{0}] elapsed {1}s".format(
            method.__name__, elapsed_time))
        return result
    return wrapper


class BagData:
    def __init__(self, data_dir, file_id, cwave_cmdline_path, **cfg):
        logger.info("Reading bag from %s" % file_id)
        self.cfg = {
            "path_period": 1.0,
        }
        self.file_id = file_id
        self.data_dir = data_dir
        self.__still_position_error = 0.051
        self.cfg.update(cfg)
        self.cwave_cmdline_path = cwave_cmdline_path
        self.bag_path = '{0}/{1}/{2}.bag'.format(data_dir, file_id, file_id)
        self.pdf_topic = "/pdf"
        self.coordinator = "/node_mediator"
        self.opdf_topic = "/opdf"
        self.getExpInfo(file_id)
        if self.experiment_info['control_method'] == 'novel':
            self.current_pose_topic = "/pose_current"
        elif self.experiment_info['control_method'] == 'steer':
            self.current_pose_topic = "/amcl_pose"
        self.readMetaInfo(self.bag_path)
        self.readData(self.bag_path)
        #logger.debug(self.meta['overall_time'])
    def getExpInfo(self, file_id):
        #parse file_id   example:peng-rob-novel-button-c2a-1-2019_01_30_20_41_17
        tag_list = file_id.split('-')
        self.experiment_info = {
            'user_id': tag_list[0],
            'environment': tag_list[1],
            'control_method': tag_list[2],
            'user_interface': tag_list[3],
            'path': tag_list[4],
            'timestamp': datetime.strptime(tag_list[6], "%Y_%m_%d_%H_%M_%S")
            #datetime.strptime(tag_list[6], "%Y_%m_%d_%H_%M_%S")
        }
        self.experiment_info['start_pose'] = self.experiment_info['path'].split('2')[0]
        self.experiment_info['goal_pose'] = self.experiment_info['path'].split('2')[1]

    #Return data needed for both statistical and single data analysis
    def getRecord(self, *args):
        record = {
            'user': self.experiment_info['user_id'],
            'timestamp': self.experiment_info['timestamp'],
            'time_to_goal': self.meta['overall_time'],
            'goal_position_error': self.getDestinationAccuracy()[0],
            'goal_orientation_error': self.getDestinationAccuracy()[1],
            'detected_cmd_number': self.meta['detected_cmd_number'],
            'path': self.calcPath(),
            'dist' : self.calcObstDist(),
            'file_id': self.file_id,
            'total_travel_distance': self.getTotalDistance(),
        }
        if self.experiment_info['control_method'] == 'novel':
            record['entropy'] = self.entropy
            record['ang_entropy'] = self.ang_entropy
            self.meta['position_arrived_time'] = self.getPositionArrivalTime()
            record['ang_dist'] = self.calcAngDist(),
        #only experiment for c2a path has workload value
        if self.experiment_info['path'] == 'c2a':
            survey_file = self.data_dir+'/../assessment/'+self.file_id+'.csv'
            if os.path.isfile(survey_file):
                record['workload'] = self.getWorkload(survey_file)
            else:
                logger.warning(
                    'Workload survey not found for {0}'.format(self.file_id))
        return record

    def readMetaInfo(self, bag_path):
        logger.info("Reading meta information")
        meta = {}
        meta['detected_cmd_number'] = 0

        #set start and goal pose
        predefine_pose = dict(a=(11.8,72.09,1.57),b=(22.88,73.72,-1.73),c=(47.75,53.11,0.92))
        meta['start_pose'] = predefine_pose[self.experiment_info['start_pose']]
        meta['goal_pose']  = predefine_pose[self.experiment_info['goal_pose']]
        for topic, msg, t in rosbag.Bag(bag_path).read_messages():
            #Read all important information from parameters here and store them in meta
            if topic == "/parameters":
                self.prms = ast.literal_eval(msg.data)
                #print self.prms
                if self.experiment_info['control_method'] == 'novel':
                    meta['interface_matrix'] = self.prms['novelti_shared_control']['interface_matrix']
            # if "map" not in meta and topic=="/map":
            #     meta['width'] = msg.info.resolution*msg.info.width
            #     meta['height'] = msg.info.resolution*msg.info.height
            # if "map_inflated" not in meta and topic=="/map_inflated":
            #     logger.debug("reading map_inflated")
            #     meta['map_inflated'] = self.makeMapInflated(msg)
            #     meta['map_inflated_resolution'] = msg.info.resolution
            if topic == "/displayed_pose_"+self.experiment_info['start_pose']:
                pose = msg.pose
                (r, p, y) = tf_conversions.transformations.euler_from_quaternion(
                    [pose.orientation.x,
                     pose.orientation.y,
                     pose.orientation.z,
                     pose.orientation.w])
                angle = y
                meta['start_pose'] = (pose.position.x, pose.position.y, angle)
            if topic == "/displayed_pose_"+self.experiment_info['goal_pose']:
                pose = msg.pose
                (r, p, y) = tf_conversions.transformations.euler_from_quaternion(
                    [pose.orientation.x,
                     pose.orientation.y,
                     pose.orientation.z,
                     pose.orientation.w])
                angle = y
                meta['goal_pose'] = (pose.position.x, pose.position.y, angle)
            if "first_pdf_stamp" not in meta and topic == self.pdf_topic:
                meta['first_pdf_stamp'] = msg.header.stamp
            if "first_opdf_stamp" not in meta and topic == self.opdf_topic:
                meta['first_opdf_stamp'] = msg.header.stamp
            if topic == "/pose_inferred":
                meta['pose_inferred_stamp'] = msg.header.stamp
                meta['pose_inferred'] = msg.pose
            if topic == "/position_inferred":
                meta['position_inferred_stamp'] = msg.header.stamp
                meta['position_inferred'] = msg.pose
            if "pose_inferred_stamp" in meta and topic == "/rosout" and msg.name == self.coordinator:
                if msg.msg == "Arrived to the destination":
                    pass
                elif msg.msg.startswith("Didn't arrive to destination"):
                    logger.warn(
                        "Failed to arrive to the inferred destination: %s" % msg.msg)
                else:
                    continue
                meta['pose_arrived_stamp'] = msg.header.stamp
                #meta['rotation_time'] = (msg.header.stamp - meta['dstArrivalStamp']).to_sec()
                meta['overall_time'] = (
                    msg.header.stamp - meta['first_pdf_stamp']).to_sec()
            if topic == '/cmd_detected':
                meta['detected_cmd_number'] += 1
            if "first_pose_stamp" not in meta and topic == self.current_pose_topic:
                meta['first_pose_stamp'] = msg.header.stamp
        self.meta = meta

    #read entropy and pose
    def readData(self, bag_path):
        logger.info("Reading data")
        self.entropy = {'t': [], 'v': []}
        self.ang_entropy = {'t': [], 'v': []}
        self.poses = {'t': [0.0], 'x': [self.meta['start_pose'][0]], 'y': [
            self.meta['start_pose'][1]], 'a': [self.meta['start_pose'][2]]}
        for topic, msg, t in rosbag.Bag(bag_path).read_messages(topics=[self.current_pose_topic, self.opdf_topic, self.pdf_topic]):
            #calculate position pdf entropy
            if self.experiment_info['control_method'] == 'novel':
                #record pose information
                if topic == self.current_pose_topic and (msg.header.stamp >= self.meta['first_pdf_stamp'] and msg.header.stamp <= self.meta['pose_arrived_stamp']):
                    #print "first pose in path has time stamp = : %f" % msg.header.stamp.to_sec()
                    t = (msg.header.stamp -
                         self.meta['first_pdf_stamp']).to_sec()
                    self.poses['t'].append(t)
                    self.poses['x'].append(msg.pose.position.x)
                    self.poses['y'].append(msg.pose.position.y)
                    (r, p, y) = tf_conversions.transformations.euler_from_quaternion(
                        [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
                    self.poses['a'].append(y)

                #calculate orientation pdf entropy
                if topic == self.opdf_topic and msg.header.stamp >= self.meta['first_opdf_stamp'] and msg.header.stamp <= self.meta['pose_arrived_stamp']:
                    t = (msg.header.stamp -
                         self.meta['first_pdf_stamp']).to_sec()
                    if self.ang_entropy['t']:
                        self.ang_entropy['t'].append(t)
                        self.ang_entropy['v'].append(self.ang_entropy['v'][-1])
                    self.ang_entropy['t'].append(t)
                    self.ang_entropy['v'].append(self.calcOPdfEntropy(msg))

                if topic == self.pdf_topic and msg.header.stamp >= self.meta['first_pdf_stamp'] and msg.header.stamp <= self.meta['position_inferred_stamp']:
                    t = (msg.header.stamp -
                         self.meta['first_pdf_stamp']).to_sec()
                    if self.entropy['t']:
                        self.entropy['t'].append(t)
                        self.entropy['v'].append(self.entropy['v'][-1])
                    self.entropy['t'].append(t)
                    self.entropy['v'].append(self.calcPdfEntropy(msg))
            #read poses from steering control experiment data
            elif self.experiment_info['control_method'] == 'steer':
                if topic == self.current_pose_topic:
                    t = (msg.header.stamp -
                         self.meta['first_pose_stamp']).to_sec()
                    self.poses['t'].append(t)
                    self.poses['x'].append(msg.pose.pose.position.x)
                    self.poses['y'].append(msg.pose.pose.position.y)
                    (r, p, y) = tf_conversions.transformations.euler_from_quaternion(
                        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                    self.poses['a'].append(y)
                    self.meta['overall_time'] = t

        self.meta['end_pose'] = (
            self.poses['x'][-1], self.poses['y'][-1], self.poses['a'][-1])

    def calcPdfEntropy(self, pdf):
        e = 0.0
        for p in pdf.data:
            if p > 0:  # if p==0,   p*log(p,2) tends to 0
                e += -p*log(p, 2)
        return e

    def calcOPdfEntropy(self, opdf):
        e = 0.0
        for p in opdf.data:
            if p > 0:  # if p==0,   p*log(p,2) tends to 0
                e += -p*log(p, 2)
        return e

    #seems like interpolating a path
    def calcPath(self):
        dt = self.cfg["path_period"]
        next_t = dt
        i = 0

        path = {'t': [0.0], 'x': [self.poses['x'][0]], 'y': [
            self.poses['y'][0]], 'a': [self.poses['a'][0]]}
        for k, t in enumerate(self.poses['t']):
            #print "===== k=%d,   t=%f,   next_t=%f" % (k,t, next_t)
            while t >= next_t:
                c = (next_t - self.poses['t'][k-1])/(t - self.poses['t'][k-1])
                i += 1
                x = self.poses['x'][k-1] + c * \
                    (self.poses['x'][k]-self.poses['x'][k-1])
                y = self.poses['y'][k-1] + c * \
                    (self.poses['y'][k]-self.poses['y'][k-1])
                #print "k=%d,    c=%f     y[k-1]=%f, y[k]=%f, " % (k, c, self.poses['y'][k-1], self.poses['y'][k])
                a0 = self.poses['a'][k-1]
                a1 = self.poses['a'][k]
                #print "%d: a0=%f, a1=%f" %(i, a0, a1)
                if (a0 > 0 and a1 < 0) or (a1 > 0 and a0 < 0):
                    if a0 > 0:
                        a0, a1 = a1, a0
                    if a1-a0 > pi:
                        a0 += 2*pi
                a = a0 + c*(a1-a0)
                #print "%d: k=%d,  t0=%f, t=%f,      c=%f,     a0=%f, a1=%f,    a=%f" %(i, k, self.poses['t'][k-1], t, c, a0, a1, a)
                path['t'].append(next_t)
                path['x'].append(x)
                path['y'].append(y)
                path['a'].append(a)
                next_t += dt
        return path

    @timer
    def getTotalDistance(self):
        total_dist = 0.
        for index in range(len(self.poses['t'])-1):
            cur_pose = np.array((self.poses['x'][index],self.poses['y'][index]))
            next_pose = np.array((self.poses['x'][index+1],self.poses['y'][index+1]))
            total_dist += np.sqrt(np.sum((next_pose - cur_pose)**2))

        return total_dist

    def getPositionArrivalTime(self):
        start_time = (self.meta['position_inferred_stamp'] - self.meta['first_pdf_stamp']).to_sec()
        for index in range(len(self.poses['t'])):
            if self.poses['t'][index] >= start_time:
                if self.isNearGoal((self.poses['x'][index],self.poses['y'][index])):
                    logger.debug('arrived position:{0}'.format([self.poses['x'][index],self.poses['y'][index]]))
                    return self.poses['t'][index]
        return None

    def isNearGoal(self, pose):
        goal = (self.poses['x'][-1],self.poses['y'][-1])
        if sqrt((goal[0]-pose[0])**2 + (goal[1]-pose[1])**2) < self.__still_position_error:
            return True
        return False

    def pose2vertex(self, x, y):
        global map_inflated_resolution
        return ( int(round(x / map_inflated_resolution)),  int(round(y / map_inflated_resolution))) 

    @timer
    def calcObstDist(self): 
        global map_inflated_string, map_inflated_resolution
        #This function is ugly, probably this whole class should be written in C++
        
        #prepare command line parameters to calculate CWave distance
        logger.info("Calculating obstacle distance")
        goal = self.meta['goal_pose']

        src = [str(v) for v in self.pose2vertex(goal[0],goal[1])]
        pts = [i for sub in   [self.pose2vertex(self.poses['x'][k], self.poses['y'][k])  for k in xrange(len(self.poses['x']))]    for i in sub]
        popen_cmd = [self.cwave_cmdline_path] + ["one2many"] + list(src) + [str(val) for val in pts]
        
        #prepare map
        # map_h,map_w = self.meta['map_inflated'].shape
        # map_data = [GridMap.FREE if cell==1.0 else GridMap.OCCUPIED  for cell in self.meta["map_inflated"].reshape(1, map_w*map_h)[0]]
        # gridMap = GridMap(map_w,map_h,map_data)
        # mapIoObj = StringIO.StringIO()
        # gridMap.printAsText(mapIoObj)
        # mapString = mapIoObj.getvalue()
        
        #print mapString


        #run CWave to calculate distances
        p = Popen(popen_cmd, stdout=PIPE, stdin=PIPE, stderr=PIPE)
        #print " ".join(popen_cmd)
        stdout_data = p.communicate(input=map_inflated_string)[0]
        #print stdout_data
        if p.returncode != 0:
            #print "Return code of cwave_cmdline tool call is non-zero. It's %d" %p.returncode
            raise RuntimeError("Return code of cwave_cmdline tool call is non-zero. It's %d" %p.returncode)
        
        # form dist array
        dist = { 
            't': self.poses['t'], 
            'v': [map_inflated_resolution*float(s) for s in stdout_data.split()]
        }

        # delete the poses where no paths can be found by cwave
        for index in range(len(dist['t'])):
            if dist['v'][index] == -1:
                logger.debug('     deleted[{0},{1}]'.format(dist['v'][index], dist['t'][index]))
                del dist['v'][index]
                del dist['t'][index]

        #print dist
        return dist

    def calcAngDist(self):
        logger.info("Calculating angular distance")
        goal = self.meta['goal_pose']
        self.orientations = {'t':[],'v':[]}
        goal_angle = goal[2]
        logger.debug('goal angle: {0}'.format(goal_angle))
        assert self.meta['position_arrived_time'], "No information for position arrived time"
        for index in range(len(self.poses['t'])):
            if self.poses['t'][index] < self.meta['position_arrived_time']:
                continue
            self.orientations['t'].append(self.poses['t'][index])
            self.orientations['v'].append(self.poses['a'][index])
        ang_dist = {
            't': self.orientations['t'],
            'v': [self.calcAngDiff(goal_angle,e) for e in self.orientations['v']]
        }
        return ang_dist
    
    def calcAngDiff(self, inferred, current):
        return abs(inferred - current) if abs(inferred - current) < pi else 2*pi - abs(inferred - current)

    @timer
    def makeMap(self, msg):
        m = np.full((msg.info.height, msg.info.width), 1)
        for x in xrange(msg.info.width):
            for y in xrange(msg.info.height):
                if msg.data[x + y*msg.info.width] > 0:
                    m[y, x] = 0
        return m

    @timer
    def makeMapInflated(self, msg):
        m = np.full((msg.info.height, msg.info.width), 0)
        for x in xrange(msg.info.width):
            for y in xrange(msg.info.height):
                if msg.data[x + y*msg.info.width] == 255:
                    m[y, x] = 1.0
        return m

    @timer
    def getMap(self):
        logger.info("Reading map")
        exp_map = {}
        for topic, msg, t in rosbag.Bag(self.bag_path).read_messages(topics=['/map', '/map_inflated']):
            if topic == "/map":
                exp_map['map'] = self.makeMap(msg)
                exp_map['width'] = msg.info.resolution*msg.info.width
                exp_map['height'] = msg.info.resolution*msg.info.height
            if topic == "/map_inflated":
                exp_map['map_inflated'] = self.makeMapInflated(msg)
                exp_map['map_inflated_resolution'] = msg.info.resolution
        return exp_map

    def getDestinationAccuracy(self):
        goal_pose = np.array(self.meta['goal_pose'])
        end_pose = np.array(self.meta['end_pose'])

        position_error = np.sqrt(np.sum((goal_pose[0:2] - end_pose[0:2])**2))
        orientation_error = np.sqrt(np.sum((goal_pose[-1] - end_pose[-1])**2))
        return [position_error, orientation_error]

    def getWorkload(self, survey_file):
        with open(survey_file) as csvfile:
            reader = csv.reader(csvfile)
            count = 0
            result = 0.
            for row in reader:
                if count >= 6:
                    break
                result += float(row[1])
                count += 1
        return result


''' 
Database stores all calculated experiment data, classified based on control method and path
Sample data:
{'user':'rui', 'timestamp':'2019_01_30_20_04_19', 'time_to_goal':'123.6', 'goal_error':'10.5', 'workload':'12', 'command_number':'60'}
'''


class DataBase:
    def __init__(self, bag_dir, cache_path, cwavetool_path):
        self.data = {
            'novel-emotiv': {'a2b': [], 'b2c': [], 'c2a': []},
            'novel-button': {'a2b': [], 'b2c': [], 'c2a': []},
            'steer-emotiv': {'a2b': [], 'b2c': [], 'c2a': []},
        }
        self.shared_data = {}
        self.bag_list = []
        self.bag_dir = bag_dir
        self.cache_path = cache_path
        self.cwavetool_path = cwavetool_path
        #Read exp_data from bag_dir to initialize the database
        for bag_name in os.listdir(self.bag_dir):
            assert self.readEntry(
                bag_name), "Failed to read {0}".format(bag_name)
            with open(self.cache_path, 'wb') as output:
                logger.debug("Saving data {0} to cache".format(bag_name))
                self.bag_list.append(bag_name)
                pickle.dump(self, output, pickle.HIGHEST_PROTOCOL)

    def updateData(self):
        logger.info("Updating data into cache %s" % self.cache_path)
        self.saveSharedData()
        for bag_name in os.listdir(self.bag_dir):
            if bag_name not in self.bag_list and (bag_name != 'database.pickle'):
                assert self.readEntry(
                    bag_name), "Failed to read {0}".format(bag_name)
                with open(self.cache_path, 'wb') as output:
                    self.bag_list.append(bag_name)
                    pickle.dump(self, output, pickle.HIGHEST_PROTOCOL)

    @timer
    def readEntry(self, bag_name):
        bag_data = BagData(self.bag_dir, bag_name, self.cwavetool_path)
        #user_id = bag_data.experiment_info['user_id']
        path = bag_data.experiment_info['path']
        control = bag_data.experiment_info['control_method'] + \
            '-'+bag_data.experiment_info['user_interface']
        #timestamp = bag_data.experiment_info['timestamp']

        if 'exp_map' not in self.shared_data:
            self.shared_data['exp_map'] = bag_data.getMap()
            self.saveSharedData()
        self.data[control][path].append(bag_data.getRecord())
        return True

    def saveSharedData(self):
        #prepare map
        global map_inflated_string 
        map_h,map_w = self.shared_data['exp_map']['map_inflated'].shape
        map_data = [GridMap.FREE if cell==1.0 else GridMap.OCCUPIED  for cell in self.shared_data['exp_map']['map_inflated'].reshape(1, map_w*map_h)[0]]
        gridMap = GridMap(map_w,map_h,map_data)
        mapIoObj = StringIO.StringIO()
        gridMap.printAsText(mapIoObj)
        map_inflated_string = mapIoObj.getvalue()

        #save map resolution
        global map_inflated_resolution
        map_inflated_resolution = self.shared_data['exp_map']['map_inflated_resolution']



    def queryByFileId(self, file_id):
        for _, path in self.data.items():
            for _, entry_list in path.items():
                for entry in entry_list:
                    if entry['file_id'] == file_id:
                        return entry
        logger.error("Entry {0} doesn't exist!".format(file_id))

    def deleteByFileId(self, file_id):
        for method, path in self.data.items():
            for path_id, entry_list in path.items():
                for entry in entry_list:
                    if entry['file_id'] == file_id:
                        logger.info("Found entry {0}, deleting".format(file_id))
                        self.data[method][path_id].remove(entry)
                        self.bag_list.remove(file_id)
                        with open(self.cache_path, 'wb') as output:
                            pickle.dump(self, output, pickle.HIGHEST_PROTOCOL)
                            return True
        logger.error("Entry {0} doesn't exist!".format(file_id))
        return False

if __name__ == "__main__":
    cache_path = data_dir+'/database.pickle'

    if os.path.isfile(cache_path):
        logger.info("Reading data from cache file %s" % cache_path)
        with open(cache_path, 'rb') as cache:
            database = pickle.load(cache)
            database.updateData()
    else:
        logger.info("No cache file found, start building a new one")
        database = DataBase(data_dir, cache_path, cwavetool_path)
