# ROS node for data logging

This ROS package performs data recording in a rosbag, with dynamic opening/closing of bag files. It was developed to be used in an Autonomous Surface Vehicle ([AWA-SV](https://github.com/disaster-robotics-proalertas/awa-sv)), and thus it monitors the vehicle state (will only record if state is "recording").

## Parameters

* log_path: Indicates the path to which the bag files will be recorded
* topics: List of topics to be recorded (ignored if record_all_topics is true)
* record_all_topics: Flag indicating whether to record all currently advertised topics
