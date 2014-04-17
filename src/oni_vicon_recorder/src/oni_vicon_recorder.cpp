

#include "oni_vicon_recorder/oni_vicon_recorder.hpp"

using namespace oni_vicon_recorder;

OniViconRecorder::OniViconRecorder(std::string name, ros::NodeHandle& node_handle):
    node_handler_(node_handle),
    oni_recorder_(node_handle),
    record_as_(node_handle, "start_" + name, boost::bind(&OniViconRecorder::recordCB, this, _1), false)
{

}

OniViconRecorder::~OniViconRecorder()
{

}

void OniViconRecorder::run()
{
    record_as_.start();

    ROS_INFO("OniViconRecorder is running ");

    ros::spin();

    ROS_INFO("Shutting down OniViconRecorder");
}


void OniViconRecorder::recordCB(const RecordGoalConstPtr& goal)
{
    ROS_INFO("Start ONI recording");
    ROS_INFO(" - Recording Name: %s", goal->name.c_str());
    ROS_INFO(" - Recording Destination %s/%s", goal->destination.c_str(), goal->name.c_str());

    RecordFeedback feedback;
    RecordResult result;

    result.vicon_frames = 0;
    result.kinect_frames = 0;

    ROS_INFO_NAMED("OniViconRecorder", "ONI recorder");
    if(!oni_recorder_.startRecording(goal->destination + "/" + goal->name + ".oni"))
    {
        ROS_WARN("ONI recording aborted.");
        record_as_.setAborted(result);
        return;
    }

    feedback.vicon_frames = 0;
    feedback.kinect_frames = 0;
    ros::Rate r(30);
    while (true)
    {
        if (record_as_.isPreemptRequested())
        {
            break;
        }
        else if (!oni_recorder_.isRecording()) // !ros::ok() ||
        {
            ROS_WARN("ONI recording aborted");
            result.vicon_frames = 0;
            result.kinect_frames = oni_recorder_.countFrames();
            record_as_.setAborted(result);
            oni_recorder_.stopRecording();
            return;
        }

        feedback.vicon_frames = 0;
        feedback.kinect_frames = oni_recorder_.countFrames();
        record_as_.publishFeedback(feedback);

        r.sleep();

        ROS_INFO("Check check check");
    }

    ROS_INFO("Stopping ONI recording");
    oni_recorder_.stopRecording();

    result.vicon_frames = 0;
    result.kinect_frames = oni_recorder_.countFrames();
    record_as_.setSucceeded(result);    
}
