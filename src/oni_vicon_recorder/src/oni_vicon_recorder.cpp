

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

//    if (!oni_recorder_.startDevice())
//    {
//        ROS_ERROR("ONI recording aborted. Cannot start depth sensor.");
//        record_as_.setAborted(result);
//        return;
//    }

    feedback.vicon_frames = 0;
    feedback.kinect_frames = 0;
    ros::Rate r(120);
    for (int i = 0; i < 1200; i++)
    {
        if (record_as_.isPreemptRequested())
        {
            break;
        }
        else if (!ros::ok())
        {
            ROS_INFO("ONI recording aborted");
            result.vicon_frames = i;
            result.kinect_frames = i/4;
            record_as_.setAborted(result);
            oni_recorder_.closeDevice();
            return;
        }

        feedback.vicon_frames = i;
        feedback.kinect_frames = i/4;
        record_as_.publishFeedback(feedback);

        r.sleep();
    }

    ROS_INFO("Stopping ONI recording");
    result.vicon_frames = feedback.vicon_frames;
    result.kinect_frames = feedback.kinect_frames;
    record_as_.setSucceeded(result);
    oni_recorder_.closeDevice();
}
