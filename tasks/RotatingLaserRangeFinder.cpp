/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RotatingLaserRangeFinder.hpp"

#include <mars/sim/RotatingRaySensor.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#undef LOG_FATAL
#include <base-logging/Logging.hpp>
using namespace mars;


RotatingLaserRangeFinder::RotatingLaserRangeFinder(std::string const& name)
    : RotatingLaserRangeFinderBase(name), mSensorID(0), mSensor(NULL)
{
}

RotatingLaserRangeFinder::RotatingLaserRangeFinder(std::string const& name, RTT::ExecutionEngine* engine)
    : RotatingLaserRangeFinderBase(name, engine), mSensorID(0), mSensor(NULL)
{
}

RotatingLaserRangeFinder::~RotatingLaserRangeFinder()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RotatingLaserRangeFinder.hpp for more detailed
// documentation about them.

bool RotatingLaserRangeFinder::configureHook()
{
    if (! RotatingLaserRangeFinderBase::configureHook())
        return false;
    output_t = hashType(_output_type.get());
    return true;
}
bool RotatingLaserRangeFinder::startHook()
{
    if (! RotatingLaserRangeFinderBase::startHook())
        return false;
           
    mSensorID = control->sensors->getSensorID( _name.value() );
    if( !mSensorID ){
        std::cout <<  "There is no sensor by the name of " << _name.value() << 
                " in the scene" << std::endl;
        return false;
    }

    mSensor = dynamic_cast<mars::sim::RotatingRaySensor*>(control->sensors->getSimSensor(mSensorID));
    if( !mSensor ){
        std::cerr  << "The sensor with " <<  _name.value() <<  
                " is not of the correct type (RotatingRaySensor)" << std::endl;
        return false;
    }

    mSensor->config.minDistance = _min_range.value();
    mSensor->config.maxDistance = _max_range.value();


    LOG_ERROR_S << "EnvireMars adaptation missing here";
    // TODO envire Mars misses some functionality here that should be added. 
    // switch(output_t){
    //     case eBoth:
    //         mSensor->config.provide_depthmap = true;
    //         mSensor->config.provide_pointcloud = true;
    //         break;
    //     case eDepthmap: 
    //         mSensor->config.provide_depthmap = true;
    //         mSensor->config.provide_pointcloud = false;
    //         break;
    //     case ePointcloud:
    //         mSensor->config.provide_depthmap = false;
    //         mSensor->config.provide_pointcloud = true;
    //         break;
    //     case eError:
    //         std::cerr  << "The output type in the config wrong." << std::endl;
    //         break;
    // }
    
    return true;
}
void RotatingLaserRangeFinder::updateHook()
{
    RotatingLaserRangeFinderBase::updateHook();

    // Seems Plugin is set up but not active yet, we are not sure that we 
    // are initialized correctly so retuning
    if(!isRunning()) {
        return; 
    }

    switch(output_t){
        case eBoth:
            writeDepthmap();
            writePointcloud();
            break;
        case eDepthmap: 
            writeDepthmap();
            break;
        case ePointcloud:
            writePointcloud();
            break;
        case eError:
            std::cerr  << "[RotatingLaserRangeFinder] The output type in the config is wrong. A typo or whitespace produce this." << std::endl;
            break;
    }

}

void RotatingLaserRangeFinder::errorHook()
{
    RotatingLaserRangeFinderBase::errorHook();
}
void RotatingLaserRangeFinder::stopHook()
{
    RotatingLaserRangeFinderBase::stopHook();
}
void RotatingLaserRangeFinder::cleanupHook()
{
    RotatingLaserRangeFinderBase::cleanupHook();
}

void RotatingLaserRangeFinder::update(double delta_t) {

}

output_type RotatingLaserRangeFinder::hashType (std::string const& inString) {
    LOG_INFO_S << "READ OUTPUT TYPE " << inString;
    if (inString == "depthmap") return eDepthmap;
    if (inString == "pointcloud") return ePointcloud;
    if (inString == "both") return eBoth; 
    std::cerr  << "The given output type" <<  inString <<  
        " is not known." << std::endl;
    return eError;
}

void RotatingLaserRangeFinder::writeDepthmap()
{
    base::samples::DepthMap depthMap;
    LOG_ERROR_S << "EnvireMars adaptation missing here";
    /*
    if(mSensor->getDepthMap(depthMap)) 
    {
        _laser_scans.write(depthMap);
    }
    */
}

void RotatingLaserRangeFinder::writePointcloud()
{
    base::samples::Pointcloud pointcloud;
    pointcloud.time = getTime();
    std::vector<mars::utils::Vector> data;
    if(mSensor->getPointcloud(data)) {
        // TODO Min/max is actually already part of the sensor
        std::vector<mars::utils::Vector>::iterator it = data.begin();
        for(; it != data.end(); it++) {
            int len_ray = it->norm();
            if(len_ray >= _min_range.get() && len_ray <= _max_range.get()) {
                base::Vector3d vec((*it)[0], (*it)[1], (*it)[2]);
                pointcloud.points.push_back(vec);
            }
        }
        _pointcloud.write(pointcloud);
    }
}

