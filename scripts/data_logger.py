#!/usr/bin/env python

from datetime import datetime
import rosbag
import rospy
import rostopic
import rosgraph
import socket
from system_monitor.msg import VehicleState
import threading
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

        # Get list of topics to be recorded
        self.topics = rospy.get_param("~topics")

        # Define subscribers for topics we want to record
        for topic in self.topics:
            msg_class, _, _ = rostopic.get_topic_class(topic)
            rospy.Subscriber(topic, msg_class, callback=self.topic_callback)

        # Dictionary to hold messages we want to record
        self.messages = {}

        # Recording state
        self.recording = False
        self.active = False
        
    def record_bag(self):
        bagname = "%s/%s_%s.bag" % (self.log_path, self.vehicle_name, datetime.now().strftime("%d-%m-%Y-%H-%M-%S"))
        bagfile = rosbag.Bag(bagname, 'w')
        while True:
            if not self.recording:
                break
            for topic in self.topics:
                if self.messages[topic][1]:
                    bagfile.write(topic, self.messages[topic][0])
                    self.messages[topic][1] = False

        # Close bag successfully
        bagfile.close()

    def topic_callback(self, msg):
        self.messages[msg._connection_header['topic']] = [msg, True]

    def state_callback(self, msg):
        if msg.id == 3:
            self.recording = True
        else:
            self.recording = False

    def run(self):
        # Start or stop recording
        if self.recording:
            if not self.active:
                # Run bag recording thread
                rospy.loginfo("[data_logger] Recording data to %s" % self.log_path)
                self.bagThread = threading.Thread(target = self.record_bag)
                self.bagThread.start()
                self.active = True        
        elif not self.recording and self.active:
            # Kill thread
            self.bagThread.join()
            self.active = False
            rospy.loginfo("[data_logger] Finished recording")

# "Main loop"
if __name__ == "__main__":
    dl = Logger(socket.gethostname())
    
    # Run while ros is active
    while not rospy.is_shutdown():
        dl.run()