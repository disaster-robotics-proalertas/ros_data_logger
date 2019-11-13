#!/usr/bin/env python

from datetime import datetime
import rosbag
import rospy
import rostopic
from system_monitor.msg import VehicleStatus
import os

# System status
vehicle_status = VehicleStatus()

# Callback for vehicle status
def vehicle_status_callback(msg):
    global vehicle_status
    vehicle_status = msg

def node():
    # Initialize node
    rospy.init_node('data_logger', anonymous=True)

    # Get vehicle name from parameter
    vehicle_name = rospy.get_param("/asv_description/system_name")

    # Define subscribers
    rospy.Subscriber('/%s/status' % vehicle_name, VehicleStatus, callback=vehicle_status_callback)

    # Get logging path (and create if nonexistant)
    log_path = rospy.get_param("~log_path", default="%s/log" % os.path.expanduser("~"))
    if not os.path.isfile(log_path):
        rospy.loginfo("[data_logger] Log dir %s does not exist, creating..." % log_path)
        os.mkdir(log_path)

    # Get flag indicating recording all topics or not
    record_all_topics = rospy.get_param("~record_all_topics", default=False)

    # Get list of topics to be recorded
    if record_all_topics:
        topics = rospy.get_published_topics()
    else:
        topics = rospy.get_param("~topics")

    # Run while node is active
    while not rospy.is_shutdown():
        # Record rosbag if vehicle status is RECORDING
        if vehicle_status.status == 3: 
            # Get bag filename from parameter
            bagname = "%s/%s.bag" % (log_path, datetime.now().strftime("%d-%m-%Y-%H-%M-%S"))
            # Define bag to be recorded
            bag = rosbag.Bag(bagname, 'w')
            
            # Keep recording until status is no longer 3
            rospy.loginfo("[data_logger] Recording rosbag as %s" % bagname)
            while 1:
                if not vehicle_status.status == 3:
                    break
                for topic in topics:
                    msg_class, _, _ = rostopic.get_topic_class(topic)
                    try:
                        topic_msg = rospy.wait_for_message(topic, msg_class, timeout=5.0)
                        bag.write(topic, topic_msg)
                    except rospy.ROSException:
                        pass
                    
            # Close bag when status changes
            bag.close()
            rospy.loginfo("[data_logger] Closed rosbag %s" % bagname)

    # Close bag when node is finished
    bag.close()
    rospy.loginfo("[data_logger] Quitting, closed rosbag %s" % bagname)

# "Main loop"
if __name__ == "__main__":
    node()
