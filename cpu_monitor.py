#!/usr/bin/env python
"""
    cpu monitor for ros nodes
    Copyright 2021 Zhiang Chen

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
"""
from __future__ import print_function
import rosnode
import psutil
import numpy as np
import time
import curses
import os

def check_pid(pid):
    """ Check For the existence of a unix pid. """
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    else:
        return True

def get_node_list():
    rosnode_list = []
    for n in rosnode.get_node_names():
        name = n[1:]
        if '/' in n:
            name = name.split('/')[0]
        rosnode_list.append(name)
    if 'gazebo' in rosnode_list:
        rosnode_list.remove('gazebo')
        rosnode_list.remove('gazebo_gui')
        rosnode_list.append('gzclient')
        rosnode_list.append('gzserver')
    return rosnode_list

def get_node_pid_dict(rosnode_list):
    rosnode_pid_dict = {}
    for rosnode_name in reversed(rosnode_list):
        pids = []
        for proc in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
            if rosnode_name in ' '.join(proc.info['cmdline']):
                if 'static_transform_publisher' in ' '.join(proc.info['cmdline']):
                    rosnode_list.remove(rosnode_name)
                    break
                pids.append(proc.pid)

            if rosnode_name in proc.name():
                if proc.pid not in pids:
                    pids.append(proc.pid)

        if len(pids) != 0:
            rosnode_pid_dict[rosnode_name] = pids
    return rosnode_pid_dict

def get_pid_list(rosnode_pid_dict):
    pids = rosnode_pid_dict.values()
    non_repeated_pid = []
    repeated_pid = []
    for node_pids in pids:
        for node_pid in node_pids:
            if node_pid not in non_repeated_pid:
                non_repeated_pid.append(node_pid)
            else:
                repeated_pid.append(node_pid)
    return non_repeated_pid, repeated_pid

def get_pid_node_dict(rosnode_pid_dict):
    pid_rosnode_dict = {}
    for k in rosnode_pid_dict:
        pids = rosnode_pid_dict[k]
        for pid in pids:
            if pid_rosnode_dict.get(pid) == None:
                pid_rosnode_dict[pid] = [k]
            else:
                pid_rosnode_dict[pid].append(k)
    return pid_rosnode_dict

def update_procs():
    rosnode_list = get_node_list()
    rosnode_pid_dict = get_node_pid_dict(rosnode_list)
    non_repeated_pid, repeated_pid = get_pid_list(rosnode_pid_dict)
    pid_rosnode_dict = get_pid_node_dict(rosnode_pid_dict)
    procs = [psutil.Process(pid) for pid in non_repeated_pid if check_pid(pid)]
    _ = [proc.cpu_percent() for proc in procs]
    return procs, pid_rosnode_dict

if __name__ == '__main__':

    rosnode_list = get_node_list()
    rosnode_pid_dict = get_node_pid_dict(rosnode_list)

    print('Processes and PIDs')
    print(rosnode_pid_dict)

    non_repeated_pid, repeated_pid = get_pid_list(rosnode_pid_dict)

    if len(repeated_pid) != 0:
        print("Caution: these PIDs are repeated")
        print(repeated_pid)
        time.sleep(1.)

    time.sleep(3.)

    pid_rosnode_dict = get_pid_node_dict(rosnode_pid_dict)

    procs = [psutil.Process(pid) for pid in non_repeated_pid if check_pid(pid)]
    _ = [proc.cpu_percent() for proc in procs]

    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()

    try:
        while True:
            rosnode_cpu_dict = {}
            rosnode_mem_dict = {}
            for proc in procs:
                if not check_pid(proc.pid):
                    continue
                cpu = proc.cpu_percent()
                mem = proc.memory_percent() #mem = proc.get_memory_info()[0] / float(2 ** 20)
                time.sleep(0.1)
                nodes = pid_rosnode_dict[proc.pid]
                for node in nodes:
                    if rosnode_cpu_dict.get(node) == None:
                        rosnode_cpu_dict[node] = [cpu]
                    else:
                        rosnode_cpu_dict[node].append(cpu)

                    if rosnode_mem_dict.get(node) == None:
                        rosnode_mem_dict[node] = [mem]
                    else:
                        rosnode_mem_dict[node].append(mem)
                        
            stdscr.erase()
            stdscr.addstr(0, 0, "{:<30}".format("ROSNODE") + "{:<15}".format("%CPU") + "{:<15}".format("%MEM") + str(len(rosnode_cpu_dict)))
            for i, k in enumerate(rosnode_cpu_dict):
                rosnode_str = "{:<30}".format(str(k))
                cpu_str = "{:<15}".format(str(np.sum(rosnode_cpu_dict[k])))
                mem_pct = np.sum(rosnode_mem_dict[k])
                mem_str = "{:.2f}".format(round(mem_pct, 2))
                mem_str = "{:<15}".format(mem_str)
                stdscr.addstr(i+1, 0, rosnode_str+cpu_str+mem_str)
            stdscr.refresh()
            procs, pid_rosnode_dict = update_procs()

    except KeyboardInterrupt:
        curses.echo()
        curses.nocbreak()
        curses.endwin()
        print('Interrupted!')








