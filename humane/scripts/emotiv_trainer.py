#! /usr/bin/python3

import subprocess
from pathlib import Path
import time
import yaml
import sys
from datetime import datetime
import os

cmd_list = [
  'blink',
  'left_wink',
  'right_wink',
  'looking_left',
  'looking_right',
  'raise_eyebrow',
  'furrow_eyebrow',
  'smile',
  'clench',
  'smirk_left',
  'smirk_right',
  'laugh'
]

home = str(Path.home())
cwd = os.getcwd()

if __name__ == "__main__":
    id = sys.argv[1]
    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    filename = "emotiv_trainning-{0}-{1}".format(id,timestamp)
    try:
        subprocess.check_output(["pgrep","Emotiv"])

    except subprocess.CalledProcessError:
        subprocess.Popen([cwd+'/helper_script/EmotivControlPanelwRecord.sh', filename])

    time.sleep(2)
    while (True):
        try:
            print("=========================")
            print(cmd_list)
            commands = input("Which expressions will be used?\n")
            commands = commands.split(' ')
            key_mappings = {}
            i = 0
            for command in commands:
                if command not in cmd_list:
                    raise ValueError('{0} is not in command list'.format(command))
                key_mappings[command] = i
                i = i+1
            #print(commands)
            with open(home+'/humane_data/emotiv_cmd_mapping/{0}_mapping.yaml'.format(id),'w') as outfile:
                output = {}
                output['key_mappings'] = key_mappings
                yaml.dump(output, outfile)
                print("Key mapping file saved to {0}_mapping.yaml".format(id))
            break
        except ValueError:
            print ("Please re-enter the commands")
            pass

    subprocess.call(["killall","ffmpeg"])
    subprocess.call(["killall","vlc"])
    exit(0)

    