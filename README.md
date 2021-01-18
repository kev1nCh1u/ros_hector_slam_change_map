# hector reload map

## start
    roslaunch hector_slam_launch tutorial.launch
    rosbag play -l src/slam_bag/Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag

    rostopic pub /Command std_msgs/String "Create Map"

    rostopic pub /Command std_msgs/String "Load Map"

    rostopic pub /syscommand std_msgs/String "reset"