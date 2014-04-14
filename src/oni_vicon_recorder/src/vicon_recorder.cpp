
#include "oni_vicon_recorder/vicon_recorder.hpp"

ViconRecorder::ViconRecorder(int float_precision):
    float_precision_(float_precision),
    hostname_("localhost:801"),
    multicast_address_("244.0.0.0:44801"),
    connect_to_multicast_(false),
    multicast_enabled_(false)
{

}

ViconRecorder::~ViconRecorder()
{

}


std::ofstream& ViconRecorder::beginRecord(std::ofstream& ofs)
{
    static bool first_record = true;

    if (!first_record)
    {
        ofs << "\n";
    }

    if (first_record)
    {
       first_record = false;
    }

    ofs << std::setprecision(float_precision_);
    return ofs;
}

std::ofstream& ViconRecorder::record(std::ofstream& ofs)
{
    ofs << " ";
    ofs << std::setprecision(float_precision_);
    return ofs;
}

std::ofstream& ViconRecorder::endRecord(std::ofstream& ofs)
{
    return record(ofs);
}
