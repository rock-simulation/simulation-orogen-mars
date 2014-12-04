/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsRotatingLaserRangeFinder.hpp"

#include <mars/sim/RotatingRaySensor.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

using namespace simulation;

MarsRotatingLaserRangeFinder::MarsRotatingLaserRangeFinder(std::string const& name)
    : MarsRotatingLaserRangeFinderBase(name), mSensorID(0), mSensor(NULL)
{
}

MarsRotatingLaserRangeFinder::MarsRotatingLaserRangeFinder(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsRotatingLaserRangeFinderBase(name, engine), mSensorID(0), mSensor(NULL)
{
}

MarsRotatingLaserRangeFinder::~MarsRotatingLaserRangeFinder()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsRotatingLaserRangeFinder.hpp for more detailed
// documentation about them.

bool MarsRotatingLaserRangeFinder::configureHook()
{
    if (! MarsRotatingLaserRangeFinderBase::configureHook())
        return false;
    return true;
}
bool MarsRotatingLaserRangeFinder::startHook()
{
    if (! MarsRotatingLaserRangeFinderBase::startHook())
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
    
    return true;
}

void MarsRotatingLaserRangeFinder::updateHook()
{
    MarsRotatingLaserRangeFinderBase::updateHook();

    // Seems Plugin is set up but not active yet, we are not sure that we 
    // are initialized correctly so retuning
    if(!isRunning()) {
        return; 
    }
    
    if(_pointcloud.connected() && mSensor->getPointcloud(mPoints)) {
        // Fill base pointcloud.
        base::samples::Pointcloud pointcloud;
        pointcloud.time = getTime();
        std::vector<mars::utils::Vector>::iterator it = mPoints.begin();
        for(; it != mPoints.end(); it++) {
            //int len_ray = it->norm();
            //if(len_ray >= _min_range.get() && len_ray <= _max_range.get()) {
                base::Vector3d vec((*it)[0], (*it)[1], (*it)[2]);
                pointcloud.points.push_back(vec);
            //}
        }
        _pointcloud.write(pointcloud);
    }
    
    if(_laser_scans.connected() && mSensor->getMultiLevelLaserScan(mScan)) {
        std::cout << "Write mlls" << std::endl;
        mScan.time = getTime();
        _laser_scans.write(mScan);
    }
}

void MarsRotatingLaserRangeFinder::errorHook()
{
    MarsRotatingLaserRangeFinderBase::errorHook();
}
void MarsRotatingLaserRangeFinder::stopHook()
{
    MarsRotatingLaserRangeFinderBase::stopHook();
}
void MarsRotatingLaserRangeFinder::cleanupHook()
{
    MarsRotatingLaserRangeFinderBase::cleanupHook();
}

void MarsRotatingLaserRangeFinder::update(double delta_t) {

}
