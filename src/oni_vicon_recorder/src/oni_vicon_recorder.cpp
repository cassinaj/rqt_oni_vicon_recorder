

#include "oni_vicon_recorder/oni_vicon_recorder.hpp"

#include <boost/filesystem.hpp>

using namespace oni_vicon_recorder;

OniViconRecorder::OniViconRecorder(std::string name, ros::NodeHandle& node_handle):
    node_handler_(node_handle),
    oni_recorder_(node_handle),
    vicon_recorder_(node_handle),
    global_calibration_(node_handle),
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
    ROS_INFO("Start ONI Vicon recording");
    ROS_INFO(" - Recording Name: %s", goal->name.c_str());
    ROS_INFO(" - Recording Destination %s/%s", goal->destination.c_str(), goal->name.c_str());

    RecordFeedback feedback;
    RecordResult result;

    result.vicon_frames = 0;
    result.kinect_frames = 0;

    // create directory before recording
    boost::filesystem::path destination_dir(goal->destination);
    destination_dir /= goal->name;
    if(!boost::filesystem::create_directory(destination_dir))
    {
        ROS_ERROR("ONI Vicon recording aborted. Could not create destination directory %s",
                  destination_dir.c_str());
        record_as_.setAborted(result);
        return;
    }

    boost::filesystem::path oni_file = destination_dir / (goal->name + ".oni");
    boost::filesystem::path vicon_file = destination_dir / (goal->name + ".txt");

    if(!oni_recorder_.startRecording(oni_file.string()))
    {
        ROS_WARN("ONI Vicon recording aborted.");
        record_as_.setAborted(result);
        return;
    }

    if(!vicon_recorder_.startRecording(vicon_file.string(), goal->object_name))
    {
        oni_recorder_.stopRecording();
        ROS_WARN("ONI Vicon recording aborted.");
        record_as_.setAborted(result);
        return;
    }

    feedback.duration = 0;
    feedback.vicon_frames = 0;
    feedback.kinect_frames = 0;
    ros::Rate r(30);

    gettimeofday(&starting_time_, NULL);

    while (true)
    {
        if (record_as_.isPreemptRequested())
        {
            break;
        }
        else if (!oni_recorder_.isRecording()
                 || !vicon_recorder_.isRecording()
                 || !ros::ok())
        {
            ROS_WARN("ONI Vicon recording aborted");
            result.vicon_frames = vicon_recorder_.countFrames();
            result.kinect_frames = oni_recorder_.countFrames();
            record_as_.setAborted(result);
            oni_recorder_.stopRecording();
            vicon_recorder_.stopRecording();
            return;
        }

        feedback.duration = duration();
        feedback.vicon_frames = vicon_recorder_.countFrames();
        feedback.kinect_frames = oni_recorder_.countFrames();
        record_as_.publishFeedback(feedback);

        r.sleep();
    }

    ROS_INFO("Stopping ONI Vicon recording");
    vicon_recorder_.stopRecording();
    oni_recorder_.stopRecording();

    result.vicon_frames = 0;
    result.kinect_frames = oni_recorder_.countFrames();
    record_as_.setSucceeded(result);    
}

u_int64_t OniViconRecorder::duration()
{
    timeval current_time;

    gettimeofday(&current_time, NULL);

    int secs(current_time.tv_sec - starting_time_.tv_sec);
    int usecs(current_time.tv_usec - starting_time_.tv_usec);

    if(usecs < 0)
    {
        --secs;
        usecs += 1000000;
    }

    return static_cast<u_int64_t>(secs * 1000 + usecs / 1000.0);
}
