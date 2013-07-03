/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsIMU.hpp"
#include "MarsPlugin.hpp"
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

using namespace simulation;

    MarsIMU::MarsIMU(std::string const& name)
    : MarsIMUBase(name)
{
}

MarsIMU::MarsIMU(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsIMUBase(name, engine)
{
}

MarsIMU::~MarsIMU()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsIMU.hpp for more detailed
// documentation about them.

// bool MarsIMU::configureHook()
// {
//     if (! MarsIMUBase::configureHook())
//         return false;
//     return true;
// }
bool MarsIMU::startHook()
{
    if (! MarsIMUBase::startHook())
        return false;

    node_id = control->nodes->getID( _name.value() );
    if( !node_id ){
        std::cerr << "There is no node by the name of " << _name .value() << " in the scene" << std::endl;
        return false;
    }

    rbs.initSane();
    rbs.position.setZero();

    return true;
}
void MarsIMU::updateHook()
{
    MarsIMUBase::updateHook();
}
// void MarsIMU::errorHook()
// {
//     MarsIMUBase::errorHook();
// }
void MarsIMU::stopHook()
{
    MarsIMUBase::stopHook();
}
// void MarsIMU::cleanupHook()
// {
//     MarsIMUBase::cleanupHook();
// }

void MarsIMU::update( double time )
{
    rbs.time = getTime();
    rbs.sourceFrame = _imu_frame.value();
    rbs.targetFrame = _world_frame.value();
    rbs.orientation = control->nodes->getRotation( node_id ).normalized();
    rbs.cov_orientation = base::Matrix3d::Ones() * 1e-6;

    _orientation_samples.write( rbs );
    
    rbs.position = control->nodes->getPosition( node_id );
    _pose_samples.write( rbs );
}
