

#ifndef ONI_VICON_RECORDER_ONI_VICON_RECORDER_HPP
#define ONI_VICON_RECORDER_ONI_VICON_RECORDER_HPP

#include <string>
#include <cstdlib>
#include <sys/time.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <oni_vicon_recorder/RecordAction.h>

#include "kinect.h"
#include "oni_recorder.hpp"
#include "vicon_recorder.hpp"

class OniViconRecorder
{
public:
    OniViconRecorder(std::string name, ros::NodeHandle& node_handle);
    ~OniViconRecorder();

    void recordCB(const oni_vicon_recorder::RecordGoalConstPtr& goal);

    void run();

private:
    u_int64_t duration();

private:
    ros::NodeHandle node_handler_;
    OniRecorder oni_recorder_;
    ViconRecorder vicon_recorder_;

    actionlib::SimpleActionServer<oni_vicon_recorder::RecordAction> record_as_;

    timeval starting_time_;
};

#endif

