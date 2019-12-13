#!/usr/bin/env python

import psutil
import rospy
import signal
import socket
import subprocess
from system_monitor.msg import VehicleState
import os

class Logger:
    def __init__(self, vehicle_name, **kwargs):
        rospy.init_node("data_logger", anonymous=True)

        # Set vehicle name
        self.vehicle_name = vehicle_name

        # Define subscribers
        rospy.Subscriber('/%s/vehicle/state' % self.vehicle_name, VehicleState, callback=self.state_callback)
        
        # Get logging path (and create if nonexistant)
        self.log_path = rospy.get_param("~log_path", default="%s/log" % os.path.expanduser("~"))
        if not os.path.isdir(self.log_path):
            rospy.loginfo("[data_logger] Log dir %s does not exist, creating..." % self.log_path)
            os.mkdir(self.log_path)

        # Define subscribers for topics we want to record
        self.topics = ''
        self.record_all_topics = rospy.get_param("~record_all_topics")
        if self.record_all_topics:
            self.topics += '-a'
        else:
            # Get list of topics to be recorded
            topicList = rospy.get_param("~topics")
            for topic in topicList:
                self.topics += '%s ' % topic

        # Recording state
        self.recording = False
        self.active = False

    def state_callback(self, msg):
        if msg.id == 3:
            self.recording = True
        else:
            self.recording = False
    
    def kill_proc(self, p):
        process = psutil.Process(p.pid)
        for sub_process in process.children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        # Wait for children to terminate
        p.wait()

    def run(self):
        # Start or stop recording
        if self.recording:
            if not self.active:
                # Run rosbag record process
                rospy.loginfo("[data_logger] Recording data to %s" % self.log_path)
                bagname = "%s/%s.bag" % (self.log_path, self.vehicle_name)
                cmd = '/opt/ros/kinetic/bin/rosbag record %s -o %s' % (self.topics, bagname)
                self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, shell=True)
                self.active = True        
        elif not self.recording and self.active:
            # Kill rosbag record process
            self.kill_proc(self.proc)
            self.active = False
            rospy.loginfo("[data_logger] Finished recording")

# "Main loop"
if __name__ == "__main__":
    dl = Logger(socket.gethostname())
    
    # Run while ros is active
    while not rospy.is_shutdown():
        dl.run()