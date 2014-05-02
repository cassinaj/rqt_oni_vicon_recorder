

#ifndef ONI_VICON_RECORDER_ONI_VICON_RECORDER_HPP
#define ONI_VICON_RECORDER_ONI_VICON_RECORDER_HPP

#include <string>
#include <cstdlib>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <oni_vicon_recorder/RecordAction.h>

#include <depth_sensor_vicon_calibration/depth_sensor_vicon_calibration.hpp>

#include "oni_vicon_recorder/kinect.h"
#include "oni_vicon_recorder/oni_recorder.hpp"
#include "oni_vicon_recorder/vicon_recorder.hpp"
#include "oni_vicon_recorder/frame_time_tracker.hpp"

class OniViconRecorder
{
public:
    OniViconRecorder(std::string name, ros::NodeHandle& node_handle);
    ~OniViconRecorder();

    void recordCB(const oni_vicon_recorder::RecordGoalConstPtr& goal);

    void run();

private:
    ros::NodeHandle node_handler_;
    FrameTimeTracker::Ptr frame_time_tracker_;
    OniRecorder oni_recorder_;
    ViconRecorder vicon_recorder_;
    depth_sensor_vicon_calibration::Calibration global_calibration_;

    actionlib::SimpleActionServer<oni_vicon_recorder::RecordAction> record_as_;    
};

#endif

