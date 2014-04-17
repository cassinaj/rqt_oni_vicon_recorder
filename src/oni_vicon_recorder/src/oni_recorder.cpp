
#include <boost/bind.hpp>

#include <oni_vicon_recorder/oni_recorder.hpp>

using namespace oni_vicon_recorder;



OniRecorder::OniRecorder(ros::NodeHandle &node_handle):
    node_handle_(node_handle),
    kinect_(NULL),
    recording_(false),
    running_(false),
    run_depth_sensor_as_(node_handle,
                     "run_depth_sensor",
                     boost::bind(&OniRecorder::runDepthSensorCB, this, _1),
                     false)
{
    run_depth_sensor_as_.start();
}

OniRecorder::~OniRecorder()
{

}

void OniRecorder::runDepthSensorCB(const RunDepthSensorGoalConstPtr &goal)
{    
    ROS_INFO("Starting depth sensor");

    RunDepthSensorFeedback feedback;
    RunDepthSensorResult result;

    kinect_ = kinect_alloc();

    if(kinect_init_and_start(kinect_, false, false, false) != 0)
    {
        kinect_free(kinect_);        
        run_depth_sensor_as_.setAborted(result);

        ROS_INFO("Cannot start depth sensor");
        return;
    }

    ROS_INFO("Depth sensor started");

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

    feedback.device_type = kinect_->vendor_id == XTION_VENDOR_ID ? "XTION": "Kinect";
    feedback.device_name = kinect_->device_name;
    feedback.res_x = mode.nXRes;
    feedback.res_y = mode.nYRes;
    feedback.fps = mode.nFPS;
    run_depth_sensor_as_.publishFeedback(feedback);

    running_ = true;

    while (true)
    {
        boost::shared_lock<boost::shared_mutex> lock(frameLock);

        if (run_depth_sensor_as_.isPreemptRequested())
        {
            break;
        }
        else if (!ros::ok())
        {
            closeDevice();
            running_ = false;
            run_depth_sensor_as_.setAborted(result);
            ROS_WARN("Running depth sensor aborted. ROS shutting down.");
            return;
        }

        if(kinect_capture(kinect_) != 0)
        {
            ROS_ERROR("Running depth sensor aborted. Error during capture.");
            closeDevice();
            running_ = false;
            recording_ = false;
            run_depth_sensor_as_.setAborted(result);
            return;
        }

        if (recording_)
        {
            frames++;
            ROS_INFO("Frame %d", frames);
        }
    }

    ROS_INFO("Closing depth sensor");
    closeDevice();
    running_ = false;
    run_depth_sensor_as_.setSucceeded(result);
}

bool OniRecorder::closeDevice()
{
    kinect_close(kinect_);
    kinect_free(kinect_);

    ROS_INFO("Depth sensor closed");
}

bool OniRecorder::startRecording(std::string destinationFile)
{
    boost::upgrade_lock<boost::shared_mutex> lock(frameLock);
    boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);

    if (!running_)
    {
        ROS_WARN("Cannot start recording. Depth sensor is not running.");
        return false;
    }

    recorder_ = Recorder();

    ROS_INFO("Starting ONI recording");
    ROS_INFO(" - output file: %s", destinationFile.c_str());

    CHECK_RC(recorder_.Create(*kinect_->g_context),
             "Create NX Recorder");

    CHECK_RC(recorder_.SetDestination(XN_RECORD_MEDIUM_FILE, destinationFile.c_str()),
             "Set recorder destination file");

    CHECK_RC(recorder_.AddNodeToRecording(*kinect_->g_depth),
             "Add depth generator node to recording");

    recording_ = true;
    frames = 0;

    ROS_INFO("Depth data recording started");

    return true;
}

bool OniRecorder::stopRecording()
{
    boost::upgrade_lock<boost::shared_mutex> lock(frameLock);
    boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);

    if (!recording_)
    {
        ROS_WARN("Recorder is not running.");
        return false;
    }

    CHECK_RC(recorder_.RemoveNodeFromRecording(*kinect_->g_depth),
             "Remove depth generator node from recording");

    recorder_.Release();
    ROS_INFO("Recording depth images stopped");

    recording_ = false;

    return true;
}

int OniRecorder::countFrames()
{
    return frames;
}

bool OniRecorder::isRecording()
{
    return recording_;
}
