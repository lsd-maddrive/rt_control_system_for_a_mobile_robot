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

class Config(object):
    LANGUAGE = 15
    TIME_AMOUNT = 15
    TIME_FOR_SLEEP = 15
    DESIRED_PROCESS_NAMES = list(['rosserial_python', 'gzserver', 'move_base', 'slam_gmapping', 'hector_mapping'])
    KERNEL_AMOUNT = 4

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
        for desired_pid_name in Config.DESIRED_PROCESS_NAMES:
            f = os.popen('pgrep -d, -f ' + desired_pid_name)
            new_pids_numbers = f.read().split(',')
            if len(new_pids_numbers) > 1:
                new_pid_number = int(new_pids_numbers[0])
                pid = psutil.Process(new_pid_number)
                pid.cpu_percent()
                data = dict([ ('pid_number', new_pid_number), ('data', list()), ('obj', pid) ])
                self.pids[desired_pid_name] = data
                print("- process {} with possible pids {}".format(desired_pid_name, new_pids_numbers))
            else:
                print("- process {} does not exist".format(desired_pid_name))
        print("Result process and their pid which exist:")
        for key in self.pids:
            print("- process {} with pid {}".format(key, self.pids[key]['pid_number']))
        rospy.sleep(Config.TIME_FOR_SLEEP)

    def __collect_data(self):
        startTime = rospy.get_rostime()
        for i in range(0, int(Config.TIME_AMOUNT / Config.TIME_FOR_SLEEP) + 1):
            currentTime = rospy.get_rostime()
            relativeSecs = currentTime.secs - startTime.secs
            relativeNsecs = currentTime.nsecs - startTime.nsecs
            relativeTime = relativeSecs + float(relativeNsecs)/1000000000
            self.time.append(relativeTime)
            print("\nTime: {}/{}".format(relativeTime, Config.TIME_AMOUNT))

            cpu_load = psutil.cpu_percent()
            self.total_cpu_load.append(cpu_load)
            print("Total: {}%".format(cpu_load))

            for pid_name in self.pids:
                pid = self.pids[pid_name]['obj']
                pid_load = pid.cpu_percent() / Config.KERNEL_AMOUNT
                self.pids[pid_name]['data'].append(pid_load)
                print("- {} with pid {}: {}%".format(pid_name, self.pids[pid_name]['pid_number'], pid_load))
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
    plt.show()

if __name__=="__main__":
    parser = argparse.ArgumentParser(description=
        """
        \rMonitor cpu load tool.
        \rTwo types of usage:
        \r1. Only plot data (default mode, you should choose the file):
        \rrosrun robot_software test_cpu_load.py --jdir test_results/test1
        \r2. Collect data and plot them (name of file will be generated):
        \rrosrun robot_software test_cpu_load.py --mode collect_and_plot 
        """,
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--jdir',
                        type=str,
                        help='JSON directory')
    parser.add_argument('--mode', 
                        help='Mode: [only_plot, collect_and_plot]',
                        default='only_plot')
    parser.add_argument('--language', 
                        help='Variants: [rus, eng]',
                        default='rus')
    parser.add_argument('--time', 
                        help='Time of cpu usage monitoring',
                        default=15)
    parser.add_argument('--interval', 
                        help='Interval between cpu measurements',
                        default=1)
    args = vars(parser.parse_args())

    try:
        rospy.init_node(THIS_NODE_NAME)
        file_path = args['jdir']
        Config.LANGUAGE = args['language']
        Config.TIME_AMOUNT = float(args['time'])
        Config.TIME_FOR_SLEEP = float(args['interval'])
        if args['mode'] == "collect_and_plot":
            data_collector = DataCollector()
            file_path = data_collector.process_collection()
        create_plot(file_path)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.logerr('Exception catched')
