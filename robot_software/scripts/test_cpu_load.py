#!/usr/bin/env python
# coding: utf8
from __future__ import print_function
import rospy
import json
import sys
import os
import matplotlib.pyplot as plt
import psutil
import argparse
import numpy as np


# Constants
THIS_NODE_NAME = 'test_cpu_load_node'
FILE_NAME_BASE = "test_cpu_load_result_"
FILE_NAME_END = ".json"
class Json:
    """ 
        Json structure:
        {
            <process_0_name>: {<CPU_LOAD_FIELD>: list()}
            ...
            <process_n_name>: {<CPU_LOAD_FIELD>: list()}
            <total_name>: {<CPU_LOAD_FIELD>: list()}
            <TIME>: list()
        }
    """
    CPU_LOAD_FIELD = 'cpu load'
    OLD_DATA = 'data'
    MEMORY_FIELD = 'memory'
    TOTAL_NAME = 'total'
    TIME = 'time'

class Config(object):
    LANGUAGE = 15
    TIME_AMOUNT = 15
    TIME_FOR_SLEEP = 15
    DESIRED_PROCESS_NAMES = list(['rosserial_python', 'gzserver', 'move_base',
                                  'slam_gmapping', 'hector_mapping',
                                  'slam_karto'])
    KERNEL_AMOUNT = 4

class DataCollector:
    def __init__(self):
        """Init of attributes that store main data"""
        self.pids = dict()
        self.time = list()
        self.total_cpu_load = list()

    def process_collection(self):
        """Init pids, collect data and write them to a file"""
        self.__init_pids()
        self.__collect_data()
        return self.__write_to_new_file()

    def __init_pids(self):
        print("The following processes do not exist:")
        for proc_name in Config.DESIRED_PROCESS_NAMES:
            f = os.popen('pgrep -d, -f ' + proc_name)
            pids_numbers = [int(i) for i in f.read().split(',')]
            if len(pids_numbers) > 1:
                pids = list()
                for pid_num in pids_numbers:
                    pids.append(psutil.Process(int(pid_num)))
                    pids[-1].cpu_percent()

                data = dict([ ('pids_numbers', pids_numbers), ('data', list()), ('obj', pids) ])
                self.pids[proc_name] = data
            else:
                print("- {}".format(proc_name))

        print("It start to monitor the following processes:")
        for key in self.pids:
            print("- {} with pid {}".format(key, self.pids[key]['pids_numbers']))
        rospy.sleep(Config.TIME_FOR_SLEEP)

    def __collect_data(self):
        start_time = rospy.get_rostime()
        for counter in range(0, int(Config.TIME_AMOUNT / Config.TIME_FOR_SLEEP) + 1):
            current_time = rospy.get_rostime()
            relative_secs = current_time.secs - start_time.secs
            relative_nsecs = current_time.nsecs - start_time.nsecs
            relative_time = relative_secs + float(relative_nsecs)/1000000000
            self.time.append(relative_time)
            print("\nTime: {}/{}".format(relative_time, Config.TIME_AMOUNT))

            cpu_load = psutil.cpu_percent()
            self.total_cpu_load.append(cpu_load)
            print("Total: {}%".format(cpu_load))

            for proc_name in self.pids:
                pids = self.pids[proc_name]['obj']
                proc_load = 0
                for pid in pids:
                    try:
                        proc_load += pid.cpu_percent()
                    except:
                        pids.remove(pid)
                proc_load /= Config.KERNEL_AMOUNT

                self.pids[proc_name]['data'].append(proc_load)
                print("- {} with pid {}: {}%".format(proc_name, self.pids[proc_name]['pids_numbers'], proc_load))

            rospy.sleep(Config.TIME_FOR_SLEEP)

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
        if key == Json.OLD_DATA:
            print("- {} mean cpu load is {}.".format(key, np.mean(dump[key])))
        elif(key != Json.TIME):
            print("- {} mean cpu load is {}.".format(key, np.mean(dump[key][Json.CPU_LOAD_FIELD])))
        else:
            print("- {} up to {}.".format(key, dump[key][-1]))

    # Create plot and show parsed data
    for key in dump:
        if key == Json.OLD_DATA:
            plt.plot(time, dump[key])
        elif key != Json.TIME:
            plt.plot(time, dump[key][Json.CPU_LOAD_FIELD])

    TITLE = ''
    X_LABEL = ''
    Y_LABEL = ''
    if Config.LANGUAGE is 'rus':
        TITLE = u'загрузка процессора'
        X_LABEL = u'время, сек'
        Y_LABEL = u'загрузка процессора, %'
    if Config.LANGUAGE is 'eng':
        TITLE = 'cpu load'
        X_LABEL = 'time, sec'
        Y_LABEL = 'cpu load, %'

    plt.title(TITLE)
    plt.xlabel(X_LABEL)
    plt.ylabel(Y_LABEL)
    plt.grid()
    plt.legend(dump.keys())
    plt.show()

if __name__=="__main__":
    parser = argparse.ArgumentParser(description=
        """
        \rMonitor cpu load tool.
        \rTwo types of usage:
        \r1. Only plot data (default mode, you should choose the file):
        \rrosrun robot_software test_cpu_load.py --jdir test_results/test.json
        \r2. Collect data and plot them (name of file will be generated):
        \rrosrun robot_software test_cpu_load.py --mode collect_and_plot 
        """,
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--jdir',
                        type=str,
                        help='JSON directory',
                        default=str())
    parser.add_argument('--mode',
                        type=str,
                        help='Mode: [only_plot, collect_and_plot]',
                        default='only_plot')
    parser.add_argument('--language',
                        type=str,
                        help='Variants: [rus, eng]',
                        default='rus')
    parser.add_argument('--time',
                        type=float,
                        help='Time of cpu usage monitoring',
                        default=15)
    parser.add_argument('--interval',
                        type=float,
                        help='Interval between cpu measurements',
                        default=1)
    args = vars(parser.parse_args())

    try:
        file_path = args['jdir']
        Config.LANGUAGE = args['language']
        Config.TIME_AMOUNT = float(args['time'])
        Config.TIME_FOR_SLEEP = float(args['interval'])
        if args['mode'] == "collect_and_plot":
            rospy.init_node(THIS_NODE_NAME)
            data_collector = DataCollector()
            file_path = data_collector.process_collection()
        create_plot(file_path)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.logerr('Exception catched')
