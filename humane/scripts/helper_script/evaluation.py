#!/usr/bin/python3
import random
import sys
from pathlib import Path
import logging
import subprocess

home = str(Path.home())
#color tag for terminal output
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

'''
Mental Demand: How much mental and perceptual activity was required (e.g. thinking, deciding, calculating, remembering, looking, searching, etc)?
Was the task easy or demanding, simple or complex, exacting or forgiving?

Physical Demand: How much physical activity was required (e.g. pushing, pulling, turning, controlling, activating, etc.)? Was the task easy or 
demanding, slow or brisk, slack or strenuous, restful or laborious?

Temporal Demand: How much time pressure did you feel due to the rate or pace at which the task or task elements occurred? Was the pace slow and 
leisurely or rapid and frantic?

Performance: How successful do you think you were in accomplishing the goals of the task set by the experimenter (or yourself)? How satisified were
you with your performance in accomplishing these goals?

Effort: How hard did you have to work (mentally and physically) to accomplish your level or performance?

Frustration level: How insecure, discouraged, irritated, streesed and annoyed versus secure, gratified, content, relaxed and complacent did you
feel during the task?
'''
rating_scale = ['Mental Demand', 'Physical Demand',
                'Temporal Demand', 'Performance', 'Effort', 'Frustration Level']
work_load = {}
result = {}

def generate_random_pair_list():
    pair_list = []
    for i in range(len(rating_scale)):
        for k in range(i+1,len(rating_scale)):
            pair_list.append([rating_scale[i],rating_scale[k]])
    
    #shuffle the pair list
    random.shuffle(pair_list)
    return pair_list

def comapre_source_of_load(pair_list):
    for pair in pair_list:
        print("Choose which of the following item was more important to your experience:(Please input index)\n1.{0} vs 2.{1}".format(pair[0],pair[1]))
        while True:
            try:
                choice = int(input('-->  '))
                if (choice >= 1 and choice <= 2):
                    break
                print("Please input an integer between 1 and 2")
            except ValueError:
                print("Please input an integer between 1 and 2")
        work_load[pair[choice-1]] += 1


def rate_weight():
    for scale in rating_scale:
        if scale == 'Performance':
            print('Please specify an integer from 0-20 (from Good to Poor) to rate the\n'
                    + bcolors.OKGREEN+' Performance '+bcolors.ENDC+'of your experience:')
        else:
            print('Please specify an integer from 0-20 (from Low to High) to rate the \n {0} {1} {2} of your experience:'.format(bcolors.OKGREEN,scale,bcolors.ENDC))
        while True:
            try:
                weight = int(input('-->  '))
                if (weight >=0 and weight <=20):
                    break
                print("Please input an integer from 0 to 20")
            except ValueError:
                print("Please input an integer from 0 to 20")
        work_load[scale] *= weight

if __name__ == "__main__":
    log = {}
    log['ID'] = sys.argv[1]
    log['Result'] = {}
    for scale in rating_scale:
        work_load[scale] = 0.
    pair_list = generate_random_pair_list()
    comapre_source_of_load(pair_list)
    rate_weight()

    for key,value in work_load.items():
        log['Result'][key] = round(value / 15,3)
    #print(log)
    
    comment = input("Thanks for your participation! You can leave any comment if you want.\n")
    with open(home+'/humane_data/assessment/'+log['ID']+'.csv', 'w') as file:
        for key,value in log['Result'].items():
            file.write(key+','+str(value)+'\n')
        file.write('Comment,'+comment+'\n')