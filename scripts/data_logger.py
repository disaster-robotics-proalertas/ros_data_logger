#!/usr/bin/env python

from datetime import datetime
import rosbag
import rospy
import rostopic
from ros_system_monitor_msgs.msg import VehicleStatus
from os.path import expanduser

# System status
sys_status = VehicleStatus()

# Callback for vehicle status
def sys_status_callback(msg):
    global sys_status
    sys_status = msg

def node():
    # Initialize node
    rospy.init_node('data_logger', anonymous=True)

    # Get vehicle name from parameter
    vehicle_name = rospy.get_param("/ros_system_monitor/vehicle_name")

    # Define subscribers
    rospy.Subscriber('/ros_system_monitor/%s/status' % vehicle_name, VehicleStatus, callback=sys_status_callback)

    # Get bag filename from parameter
    bagname = rospy.get_param('~/log_filename', default="%s/log/%s.bag" % (expanduser("~"), datetime.now().strftime("%d-%m-%Y-%H-%M-%S")))

    # Define bag to be recorded
    bag = rosbag.Bag(bagname, 'w')
    
    # Get flag indicating recording all topics or not
    record_all_topics = rospy.get_param("~/record_all_topics", default=False)

    # Get list of topics to be recorded
    if record_all_topics:
        topics = rospy.get_published_topics()
    else:
        topics = rospy.get_param("~/topics")

    # Run while node is active
    while not rospy.is_shutdown():
        # Record rosbag if system status is RECORDING
        if sys_status.status == 3:
            for topic in topics:
                msg_class, _, _ = rostopic.get_topic_class(topic)
                topic_msg = rospy.wait_for_message(topic, msg_class)
                bag.write(topic, topic_msg)

    # Close bag when node is finished
    bag.close()

# "Main loop"
if __name__ == "__main__":
    node()