#!/usr/bin/env python
import rospy
import json
import matplotlib.pyplot as plt
import os, psutil

# Constants
THIS_NODE_NAME = 'test_cpu_load_node'
FILE_NAME_BASE = "test_cpu_load_result_"
FILE_NAME_END = ".json"

rospy.init_node(THIS_NODE_NAME)

def gatherData():
    # Gather data
    data = list()
    time = list()
    startTime = rospy.get_rostime()
    for i in range(1, 15):
        currentTime = rospy.get_rostime()
        relativeSecs = currentTime.secs - startTime.secs
        relativeNsecs = currentTime.nsecs - startTime.nsecs
        relativeTime = relativeSecs + float(relativeNsecs)/1000000000
        time.append(relativeTime)
        data.append(psutil.cpu_percent())
        rospy.sleep(0.5)
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

    plt.plot(time, data)
    plt.show()
    plt.grid()


try:
    fileName = gatherData()
    #fileName = ""
    createPlot(fileName)

    
except (rospy.ROSInterruptException, KeyboardInterrupt):
    rospy.logerr('Exception catched')
