
#include <ros/ros.h>

#include "oni_vicon_recorder/oni_vicon_recorder.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oni_vicon_recorder");
    ros::NodeHandle node_handler;

    OniViconRecorder oni_vicon_recorder(node_handler);
    oni_vicon_recorder.run();
    return 0;
}
