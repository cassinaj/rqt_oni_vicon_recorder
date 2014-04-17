
#include<boost/bind.hpp>

#include <oni_vicon_recorder/oni_recorder.hpp>

using namespace oni_vicon_recorder;

OniRecorder::OniRecorder(ros::NodeHandle &node_handle):
    node_handle_(node_handle),
    kinect_(NULL),
    start_sensor_as_(node_handle,
                     "start_depth_sensor",
                     boost::bind(&OniRecorder::startDeviceCB, this, _1),
                     false),
    close_sensor_as_(node_handle,
                     "close_depth_sensor",
                     boost::bind(&OniRecorder::closeDeviceCB, this, _1),
                     false)
{
    start_sensor_as_.start();
    close_sensor_as_.start();
}

OniRecorder::~OniRecorder()
{

}

void OniRecorder::startDeviceCB(const StartDepthSensorGoalConstPtr& goal)
{    
    ROS_INFO("Starting depth sensor");

    StartDepthSensorResult result;

    kinect_ = kinect_alloc();

    if(kinect_init_and_start(kinect_, false, false, false) != 0)
    {
        kinect_free(kinect_);

        ROS_INFO("Cannot start depth sensor device");
        result.device_type = "";
        result.device_name = "";
        result.res_x = 0;
        result.res_y = 0;
        result.fps = 0;
        start_sensor_as_.setAborted(result);
        closeDevice();

        return;
    }

    ROS_INFO("Depth sensor device started");

    XnMapOutputMode mode;
    kinect_->g_depth->GetMapOutputMode(mode);

    kinect_params_t params;
    params.rgb_focal_length    = KINECT_RGB_FOCAL_LENGTH_DEFAULT;
    params.ir_focal_length     = KINECT_IR_FOCAL_LENGTH_DEFAULT;
    params.ir_camera_center.x  = KINECT_IR_CENTER_COL_DEFAULT;
    params.ir_camera_center.y  = KINECT_IR_CENTER_ROW_DEFAULT;
    params.rgb_camera_center.x = KINECT_RGB_CENTER_COL_DEFAULT;
    params.rgb_camera_center.y = KINECT_RGB_CENTER_ROW_DEFAULT;
    kinect_set_transform( KINECT_RGB_TO_IR_X_DEFAULT,
                          KINECT_RGB_TO_IR_Y_DEFAULT,
                          KINECT_RGB_TO_IR_Z_DEFAULT,
                          KINECT_RGB_TO_IR_ROLL_DEFAULT,
                          KINECT_RGB_TO_IR_PITCH_DEFAULT,
                          KINECT_RGB_TO_IR_YAW_DEFAULT,
                          params.rgb_to_ir);
    kinect_set_params(kinect_, params);

    result.device_type = kinect_->vendor_id == XTION_VENDOR_ID ? "XTION": "Kinect";
    result.device_name = kinect_->device_name;
    result.res_x = mode.nXRes;
    result.res_y = mode.nYRes;
    result.fps = mode.nFPS;
    start_sensor_as_.setSucceeded(result);
}

void OniRecorder::closeDeviceCB(const oni_vicon_recorder::CloseDepthSensorGoalConstPtr& goal)
{
    closeDevice();
    close_sensor_as_.setSucceeded(CloseDepthSensorResult());
}

bool OniRecorder::closeDevice()
{
    kinect_close(kinect_);
    kinect_free(kinect_);
}
