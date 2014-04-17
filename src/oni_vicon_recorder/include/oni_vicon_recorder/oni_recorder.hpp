

#ifndef ONI_VICON_RECORDER_ONI_RECORDER_HPP
#define ONI_VICON_RECORDER_ONI_RECORDER_HPP

#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <oni_vicon_recorder/StartDepthSensorAction.h>
#include <oni_vicon_recorder/CloseDepthSensorAction.h>

#include "kinect.h"

#define KINECT_IMAGE_COLS       640
#define KINECT_IMAGE_ROWS       480

#define KINECT_RGB_FOCAL_LENGTH_DEFAULT     525
#define KINECT_RGB_CENTER_COL_DEFAULT       320
#define KINECT_RGB_CENTER_ROW_DEFAULT       240
#define KINECT_IR_FOCAL_LENGTH_DEFAULT      580
#define KINECT_IR_CENTER_COL_DEFAULT        320
#define KINECT_IR_CENTER_ROW_DEFAULT        240
#define KINECT_RGB_TO_IR_X_DEFAULT          (-0.0254)
#define KINECT_RGB_TO_IR_Y_DEFAULT          (-0.00013)
#define KINECT_RGB_TO_IR_Z_DEFAULT          (-0.00218)
#define KINECT_RGB_TO_IR_ROLL_DEFAULT       0.0 // rad
#define KINECT_RGB_TO_IR_PITCH_DEFAULT      0.0 // rad
#define KINECT_RGB_TO_IR_YAW_DEFAULT        0.0 // rad

#define KINECT_VENDOR_ID                    0x45e
#define XTION_VENDOR_ID                     0x1d27

class OniRecorder
{
public:


public:
    OniRecorder(ros::NodeHandle& node_handle);
    ~OniRecorder();

    void startDeviceCB(const oni_vicon_recorder::StartDepthSensorGoalConstPtr& goal);
    void closeDeviceCB(const oni_vicon_recorder::CloseDepthSensorGoalConstPtr& goal);
    bool closeDevice();

private:
    ros::NodeHandle node_handle_;
    kinect_t* kinect_;

    actionlib::SimpleActionServer<oni_vicon_recorder::StartDepthSensorAction> start_sensor_as_;
    actionlib::SimpleActionServer<oni_vicon_recorder::CloseDepthSensorAction> close_sensor_as_;
};

#endif