#!/usr/bin/env python
import rospy
import json
import sys
import os
import matplotlib.pyplot as plt
import psutil
import argparse

# Constants
THIS_NODE_NAME = 'test_cpu_load_node'
FILE_NAME_BASE = "test_cpu_load_result_"
FILE_NAME_END = ".json"

TIME_FOR_STEP = 0.5
TIME_AMOUNT = 9.5

rospy.init_node(THIS_NODE_NAME)

def gatherData():
    # Gather data
    data = list()
    time = list()
    startTime = rospy.get_rostime()
    for i in range(0, int(TIME_AMOUNT / TIME_FOR_STEP) + 1):
        currentTime = rospy.get_rostime()
        relativeSecs = currentTime.secs - startTime.secs
        relativeNsecs = currentTime.nsecs - startTime.nsecs
        relativeTime = relativeSecs + float(relativeNsecs)/1000000000
        time.append(relativeTime)
        cpu_load = psutil.cpu_percent()
        data.append(cpu_load)
        rospy.sleep(TIME_FOR_STEP)
        print(str(relativeTime) + "/" + str(TIME_AMOUNT) + ": " + str(cpu_load) + "%.")
    # Write file
    counter = 0
    while os.path.isfile(FILE_NAME_BASE + str(counter) + FILE_NAME_END) is not False:
        counter += 1
    fileName = FILE_NAME_BASE + str(counter) + FILE_NAME_END
    write_file = open(fileName, "w")
    new_dump = dict([("time", time),
                     ("data", data)])
    json.dump(new_dump, write_file, indent=2)
    print("File name is" + str(fileName))
    
    return fileName

def createPlot(fileName):
    read_file = open(fileName, "r")
    dump = json.load(read_file)
    time = dump["time"]
    data = dump["data"]

    plt.title("cpu load")
    plt.xlabel("time, sec")
    plt.ylabel("cpu load, %")
    plt.plot(time, data)
    plt.grid()
    plt.show()

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Test cpu load tool')
    parser.add_argument('--jdir',
                        type=str,
                        help='Default JSON dir',
                        default=str())
    parser.add_argument('--mode', 
                        help='Mode: [only_plot, gather_and_plot]',
                        default='only_plot')
    args = vars(parser.parse_args())

    try:
        filePath = args['jdir']
        if args['mode'] == "gather_and_plot":
            filePath = gatherData()
        createPlot(filePath)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.logerr('Exception catched')
