#!/usr/bin/env python
# coding: utf8
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
    """ 
        Json structure:
        {
            <pid_0_name>: {<CPU_LOAD_FIELD>: list()}
            ...
            <pid_n_name>: {<CPU_LOAD_FIELD>: list()}
            <total_name>: {<CPU_LOAD_FIELD>: list()}
            <TIME>: list()
        }
    """
    CPU_LOAD_FIELD = 'cpu load'
    MEMORY_FIELD = 'memory'
    TOTAL_NAME = 'total'
    TIME = 'time'


# Choose parameters
DESIRED_PROCESS_NAMES = list(['rosserial_python'])
TIME_FOR_SLEEP = 1
TIME_AMOUNT = 15
KERNEL_AMOUNT = 4
LANGUAGE = 'rus'


rospy.init_node(THIS_NODE_NAME)

class DataCollector:
    def __init__(self):
        self.pids = dict()
        self.time = list()
        self.total_cpu_load = list()

    def process_collection(self):
        self.__init_pids()
        self.__collect_data()
        return self.__write_to_new_file()

    def __init_pids(self):
        self.pids = dict()
        print("User chose following process monitoring:")
        for desired_pid_name in DESIRED_PROCESS_NAMES:
            f = os.popen('pgrep -d, -f ' + desired_pid_name)
            output = f.read().split(',')
            if len(output) > 1:
                new_pid_number = int(output[0])
                pid = psutil.Process(new_pid_number)
                pid.cpu_percent()
                data = dict([ ('pid_number', new_pid_number), ('data', list()), ('obj', pid) ])
                self.pids[desired_pid_name] = data
                print("- process {} with possible pids {}".format(desired_pid_name, output))
            else:
                print("- process {} does not exist".format(desired_pid_name))
        print("Result process and their pid which exist:")
        for key in self.pids:
            print("- process {} with pid {}".format(key, self.pids[key]['pid_number']))
        rospy.sleep(TIME_FOR_SLEEP)

    def __collect_data(self):
        startTime = rospy.get_rostime()
        for i in range(0, int(TIME_AMOUNT / TIME_FOR_SLEEP) + 1):
            currentTime = rospy.get_rostime()
            relativeSecs = currentTime.secs - startTime.secs
            relativeNsecs = currentTime.nsecs - startTime.nsecs
            relativeTime = relativeSecs + float(relativeNsecs)/1000000000
            self.time.append(relativeTime)
            print("\nTime: {}/{}".format(relativeTime, TIME_AMOUNT))

            cpu_load = psutil.cpu_percent()
            self.total_cpu_load.append(cpu_load)
            print("Total: {}%".format(cpu_load))

            for pid_name in self.pids:
                pid = self.pids[pid_name]['obj']
                pid_load = pid.cpu_percent() / KERNEL_AMOUNT
                self.pids[pid_name]['data'].append(pid_load)
                print("- {} with pid {}: {}%".format(pid_name, self.pids[pid_name]['pid_number'], pid_load))
            rospy.sleep(TIME_FOR_SLEEP)

    def __write_to_new_file(self):
        counter = 0
        while os.path.isfile(FILE_NAME_BASE + str(counter) + FILE_NAME_END) is not False:
            counter += 1
        fileName = FILE_NAME_BASE + str(counter) + FILE_NAME_END
        write_file = open(fileName, "w")

        new_dump = dict([ (Json.TIME, self.time),
                          (Json.TOTAL_NAME, dict([(Json.CPU_LOAD_FIELD, self.total_cpu_load)])) ])
        for pid_name in self.pids:
            pid = self.pids[pid_name]
            pid_cpu_load = pid['data']
            new_dump[pid_name] = dict([(Json.CPU_LOAD_FIELD, pid_cpu_load)])

        json.dump(new_dump, write_file, indent=2)
        print("File name is {}".format(fileName))
        return fileName

def create_plot(fileName):
    # Read file and check correctness
    read_file = open(fileName, "r")
    dump = json.load(read_file)
    if Json.TIME in dump:
        time = dump[Json.TIME]
    else:
        print("Json parse error: there is no time data!")
        return

    # Show data
    for key in dump:
        print("{} is {}.".format(key, dump[key]))

    # Create plot and show parsed data
    for key in dump:
        if key == Json.TIME:
            continue
        else:
            print("- {} was founded in json.".format(key))
            plt.plot(time, dump[key][Json.CPU_LOAD_FIELD])

    TITLE = ''
    X_LABEL = ''
    Y_LABEL = ''
    if LANGUAGE is 'rus':
        TITLE = u'загрузка процессора'
        X_LABEL = u'время, сек'
        Y_LABEL = u'загрузка процессора, %'
    if LANGUAGE is 'eng':
        TITLE = 'cpu load'
        X_LABEL = 'time, sec'
        Y_LABEL = 'cpu load, %'

    plt.title(TITLE)
    plt.xlabel(X_LABEL)
    plt.ylabel(Y_LABEL)
    plt.grid()
    plt.show()

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Test cpu load tool')
    parser.add_argument('--jdir',
                        type=str,
                        help='Default JSON dir',
                        default=str())
    parser.add_argument('--mode', 
                        help='Mode: [only_plot, collect_and_plot]',
                        default='only_plot')
    args = vars(parser.parse_args())

    try:
        filePath = args['jdir']
        if args['mode'] == "collect_and_plot":
            data_collector = DataCollector()
            filePath = data_collector.process_collection()
        create_plot(filePath)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.logerr('Exception catched')
