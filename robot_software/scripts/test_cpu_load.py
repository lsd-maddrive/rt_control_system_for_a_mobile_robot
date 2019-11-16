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

class Json:
    PID_DATA_NAME = 'cpu load'
    TOTAL_CPU_LOAD = 'total cpu load'
    TIME = 'time'

GATHERING_TIME = 3
TIME_FOR_SLEEP = 0.5
TIME_AMOUNT = 60

rospy.init_node(THIS_NODE_NAME)

def gatherData():
    # Define pids
    desired_pid_names = list(['rosserial_python'])
    pids = dict()
    for desired_pid_name in desired_pid_names:
        f = os.popen('pgrep -d, -f ' + desired_pid_name)
        output = f.read().split(',')
        print("process {} with pid {}".format(desired_pid_name, output))
	if len(output) > 1:
	    new_pid_number = int(output[0])
            data = dict([ ('pid_number', new_pid_number), ('data', list()) ])
            pids[desired_pid_name] = data

    # Gather data
    data = list()
    time = list()
    startTime = rospy.get_rostime()
    for i in range(0, int(TIME_AMOUNT / (GATHERING_TIME * len(pids) + TIME_FOR_SLEEP) + 1)):
        currentTime = rospy.get_rostime()
        relativeSecs = currentTime.secs - startTime.secs
        relativeNsecs = currentTime.nsecs - startTime.nsecs
        relativeTime = relativeSecs + float(relativeNsecs)/1000000000
        time.append(relativeTime)
        print("\nTime: {}/{}".format(relativeTime, TIME_AMOUNT))

        cpu_load = psutil.cpu_percent()
        data.append(cpu_load)
        print("Total: {}%".format(cpu_load))

        for pid_name in pids:
            pid = psutil.Process(pids[pid_name]['pid_number'])
	    pid_load = pid.cpu_percent(GATHERING_TIME)
            pids[pid_name]['data'].append(pid_load)
            print("- {} with pid {}: {}%".format(pid_name, pids[pid_name]['pid_number'], pid_load))
        rospy.sleep(TIME_FOR_SLEEP)

    # Write file
    counter = 0
    while os.path.isfile(FILE_NAME_BASE + str(counter) + FILE_NAME_END) is not False:
        counter += 1
    fileName = FILE_NAME_BASE + str(counter) + FILE_NAME_END
    write_file = open(fileName, "w")
    new_dump = dict([("time", time),
                     (Json.TOTAL_CPU_LOAD, data)])
    for pid_name in pids:
        pid = pids[pid_name]
        data = pid['data']
        new_dump[pid_name] = dict([('data', data)])
    json.dump(new_dump, write_file, indent=2)
    print("File name is" + str(fileName))
    
    return fileName

def createPlot(fileName):
    # Read file
    read_file = open(fileName, "r")
    dump = json.load(read_file)
    time = dump["time"]
    total_cpu_data = dump[Json.TOTAL_CPU_LOAD]

    # Show data
    for key in dump:
        print("{} is {}.".format(key, dump[key]))

    # Create plot
    #for i in range(1, len())
    #plt.subplot(2, 2, i)
    if 'time' in dump:
        if Json.TOTAL_CPU_LOAD in dump:
            plt.subplot(2, 2, 1)
            plt.title(Json.TOTAL_CPU_LOAD)
            plt.xlabel("{}, sec".format(Json.TIME))
            plt.ylabel("{}%".format(Json.TOTAL_CPU_LOAD))
            plt.plot(time, dump[Json.TOTAL_CPU_LOAD])
            plt.grid()
        else:
            print("Json parse error: there is no cpu load data!")
        plt.show()
    else:
        print("Json parse error: there is no time data!")

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
