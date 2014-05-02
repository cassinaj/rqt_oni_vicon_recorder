
#include <sys/time.h>
#include <unistd.h>

#include "oni_vicon_recorder/frame_time_tracker.hpp"

FrameTimeTracker::FrameTimeTracker():
    starting_time_(0.),
    vicon_frame_(0),
    depth_sensor_frame_(0)
{
}

FrameTimeTracker::~FrameTimeTracker()
{
}

void FrameTimeTracker::reset()
{
    vicon_frame_ = 0;
    depth_sensor_frame_ = 0;

    starting_time_ = timeInMilliseconds();
    vicon_frame_time_ = 0.;
    depth_senso_frame_time_ = 0.;
}

unsigned long FrameTimeTracker::viconFrame()
{
    return vicon_frame_ ;
}

unsigned long FrameTimeTracker::depthSensorFrame()
{
    return depth_sensor_frame_;
}

u_int64_t FrameTimeTracker::viconFrameTime()
{
    return vicon_frame_time_;
}

u_int64_t FrameTimeTracker::depthSensorFrameTime()
{
    return depth_senso_frame_time_;
}

void FrameTimeTracker::viconFrame(unsigned long vicon_frame)
{\
    vicon_frame_ = vicon_frame;
    vicon_frame_time_ = timeInMilliseconds();
}

void FrameTimeTracker::depthSensorFrame(unsigned long depth_sensor_frame)
{
    depth_sensor_frame_ = depth_sensor_frame;
    depth_senso_frame_time_ = timeInMilliseconds();
}

u_int64_t FrameTimeTracker::timeInSeconds()
{
    return timeInMilliseconds() / 1000;
}

u_int64_t FrameTimeTracker::timeInMicroseconds()
{
    return timeInMilliseconds() * 1000;
}

u_int64_t FrameTimeTracker::timeInMilliseconds()
{
    timeval current_time;
    gettimeofday(&current_time, NULL);

    return (current_time.tv_sec * 1000  + current_time.tv_usec / 1000) - starting_time_;
}
