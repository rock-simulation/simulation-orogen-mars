/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LaserRangeFinder.hpp"
#include "Plugin.hpp"
#include <mars/sim/RaySensor.h>
#include <base/Time.hpp>
#include <mars/interfaces/sim/SensorManagerInterface.h>
#include <mars/interfaces/sim/EntityManagerInterface.h>

using namespace mars;

LaserRangeFinder::LaserRangeFinder(std::string const& name)
    : LaserRangeFinderBase(name)
{
}

LaserRangeFinder::LaserRangeFinder(std::string const& name, RTT::ExecutionEngine* engine)
    : LaserRangeFinderBase(name, engine)
{
}

LaserRangeFinder::~LaserRangeFinder()
{
}

bool LaserRangeFinder::startHook()
{
    if (! LaserRangeFinderBase::startHook())
        return false;

    std::string robot_name = _robot_name.value();
    std::string sensor_name = _name.value();

    long sensor_id = 0;
    if(robot_name != "") {
        sensor_id = control->entities->getEntitySensor(robot_name, sensor_name);
    } else {
        sensor_id = control->sensors->getSensorID(sensor_name);
    }

    if( !sensor_id ){
        std::cout <<  "There is no sensor by the name of " << _name.value() << " in the scene" << std::endl;
        return false;
    }

    sensor = dynamic_cast<mars::sim::RaySensor*>( control->sensors->getSimSensor( sensor_id ) );
    if( !sensor ){
        std::cerr  << "The sensor with " <<  sensor_name <<  " is not of the correct type (RaySensor)" << std::endl;
        return false;
    }

    return true;
}

void LaserRangeFinder::stopHook()
{
    LaserRangeFinderBase::stopHook();
}

void  LaserRangeFinder::update( double time )
{
    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
    scan.time = getTime();
    std::vector<double> ranges = sensor->getSensorData();
    scan.ranges.resize( ranges.size() );
    scan.minRange = _min_range.get() * 1000;
    scan.maxRange = sensor->getConfig().maxDistance * 1000;
    for( size_t i=0; i<ranges.size(); i++ )
    {
        const long range = ranges[i] * 1000;
        scan.ranges[i] = ( range < scan.maxRange ) ? range : base::samples::TOO_FAR;
    }
    // assume scan to be centered
    if( !scan.ranges.empty() )
    {
        const double opening_width = sensor->getConfig().opening_width; 
        scan.start_angle = -opening_width / 2.0;
        scan.angular_resolution = opening_width / scan.ranges.size();
        _scans.write( scan );
    }
}
