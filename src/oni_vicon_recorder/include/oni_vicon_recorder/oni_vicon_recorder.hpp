

#ifndef ONI_VICON_RECORDER_ONI_VICON_RECORDER_HPP
#define ONI_VICON_RECORDER_ONI_VICON_RECORDER_HPP


#include <ros/ros.h>

#include "kinect.h"
#include "oni_recorder.hpp"
#include "vicon_recorder.hpp"


class OniViconRecorder
{
public:
    OniViconRecorder(ros::NodeHandle& node_handler);
    ~OniViconRecorder();

    void run();

private:
    ros::NodeHandle node_handler_;
    OniRecorder oni_recorder_;
    ViconRecorder vicon_recorder_;
};

#endif

