
#ifndef ONI_VICON_RECORDER_FRAME_TIME_TRACKER_HPP
#define ONI_VICON_RECORDER_FRAME_TIME_TRACKER_HPP

#include <boost/shared_ptr.hpp>

class FrameTimeTracker
{
public:
    typedef boost::shared_ptr<FrameTimeTracker> Ptr;

public:
    FrameTimeTracker();
    virtual ~FrameTimeTracker();

    void reset();

    long unsigned int viconFrame();
    long unsigned int depthSensorFrame();

    u_int64_t viconFrameTime();
    u_int64_t depthSensorFrameTime();

    void viconFrame(long unsigned int vicon_frame);
    void depthSensorFrame(long unsigned int depth_sensor_frame);

    u_int64_t timeInSeconds();
    u_int64_t timeInMilliseconds();
    u_int64_t timeInMicroseconds();

private:
    u_int64_t starting_time_;
    u_int64_t vicon_frame_time_;
    u_int64_t depth_senso_frame_time_;
    long unsigned int vicon_frame_;
    long unsigned int depth_sensor_frame_;
};

#endif

