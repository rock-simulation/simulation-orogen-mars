/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "IMU.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/utils/mathUtils.h>
#include <mars/interfaces/sim/ControlCenter.h>

using namespace mars;

IMU::IMU(std::string const& name)
    : IMUBase(name)
{
}

IMU::IMU(std::string const& name, RTT::ExecutionEngine* engine)
    : IMUBase(name, engine)
{
}

IMU::~IMU()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See IMU.hpp for more detailed
// documentation about them.

// bool IMU::configureHook()
// {
//     if (! IMUBase::configureHook())
//         return false;
//     return true;
// }
bool IMU::startHook()
{
    if (! IMUBase::startHook())
        return false;

    std::string robot_name = _robot_name.value();
    std::string node_name = _name.value();

    if(robot_name != "") {
        node_id = control->entities->getEntityNode(robot_name, node_name);
    } else {
        node_id = control->nodes->getID(node_name);
    }

    
    if( !node_id ){
        std::cerr << "There is no node by the name of " << node_name << " in the scene" << std::endl;
        return false;
    }

    rbs.initSane();
    rbs.position.setZero();

    if (_rotate_node_relative.get().size() == 3){


		mars::utils::Vector rotoff;// = _rotate_node_relative.get();

		rotoff.x() =  _rotate_node_relative.get()[0];
		rotoff.y() =  _rotate_node_relative.get()[1];
		rotoff.z() =  _rotate_node_relative.get()[2];


		mars::interfaces::NodeData nodedata = control->nodes->getFullNode(node_id);

		nodedata.rot = mars::utils::eulerToQuaternion(rotoff) * nodedata.rot;

		control->nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_ROT);

    }
    
    translation_noise = boost::normal_distribution<double>(0.0, _position_sigma.get());
    rotation_noise = boost::normal_distribution<double>(0.0, _orientation_sigma.get());
    velocity_noise = boost::normal_distribution<double>(0.0, _velocity_sigma.get());
    angular_velocity_noise = boost::normal_distribution<double>(0.0, _angular_velocity_sigma.get());

    return true;
}
void IMU::updateHook()
{
    IMUBase::updateHook();
}
// void IMU::errorHook()
// {
//     IMUBase::errorHook();
// }
void IMU::stopHook()
{
    IMUBase::stopHook();
}
// void IMU::cleanupHook()
// {
//     IMUBase::cleanupHook();
// }

void IMU::update( double time )
{
    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    //First make all invalid
    rbs.invalidate();

    rbs.time = getTime();
    rbs.sourceFrame = _imu_frame.value();
    rbs.targetFrame = _world_frame.value();
    if(_provide_orientation.get()){
        rbs.orientation = control->nodes->getRotation( node_id ).normalized();
        rbs.cov_orientation = base::Matrix3d::Identity() * std::max(std::pow(rotation_noise.sigma(), 2), 1e-6);
        if( rotation_noise.sigma() > 0.0 )
        {
            // apply noise to orientation
            base::Orientation orientation_error = Eigen::AngleAxisd(rotation_noise(rnd_generator), Eigen::Vector3d::UnitX())*
                                                    Eigen::AngleAxisd(rotation_noise(rnd_generator), Eigen::Vector3d::UnitY()) *
                                                    Eigen::AngleAxisd(rotation_noise(rnd_generator), Eigen::Vector3d::UnitZ());
            rbs.orientation = orientation_error * rbs.orientation;
        }
        if(_provide_velocity.get())
        {
            rbs.angular_velocity = rbs.orientation.conjugate() * control->nodes->getAngularVelocity( node_id);
            rbs.cov_angular_velocity = base::Matrix3d::Identity() * std::max(std::pow(angular_velocity_noise.sigma(), 2), 1e-6);
            if( angular_velocity_noise.sigma() > 0.0 )
            {
                // apply noise to angular velocity
                rbs.angular_velocity = rbs.angular_velocity + base::Vector3d(angular_velocity_noise(rnd_generator), angular_velocity_noise(rnd_generator), angular_velocity_noise(rnd_generator));
            }
        }
    }


    if(_provide_position.get()){
        rbs.position = control->nodes->getPosition( node_id );
        rbs.cov_position = base::Matrix3d::Identity() * std::max(std::pow(translation_noise.sigma(), 2), 1e-6);
        if( translation_noise.sigma() > 0.0 )
        {
            // apply noise to position
            rbs.position = rbs.position + base::Vector3d(translation_noise(rnd_generator), translation_noise(rnd_generator), translation_noise(rnd_generator));
        }

        if(_provide_velocity.get()){
            rbs.velocity = control->nodes->getLinearVelocity( node_id );
            rbs.cov_velocity = base::Matrix3d::Identity() * std::max(std::pow(velocity_noise.sigma(), 2), 1e-6);
            if( velocity_noise.sigma() > 0.0 )
            {
                // apply noise to velocity
                rbs.velocity = rbs.velocity + base::Vector3d(velocity_noise(rnd_generator), velocity_noise(rnd_generator), velocity_noise(rnd_generator));
            }
        }
    }

    _orientation_samples.write( rbs );

    imusens.time = getTime();
    // transform acceleration and rotation to IMU frame:
    base::Orientation orientation_con = control->nodes->getRotation( node_id ).normalized().conjugate();
    imusens.acc  = orientation_con * (control->nodes->getLinearAcceleration( node_id ) - control->sim->getGravity());
    imusens.gyro = orientation_con * control->nodes->getAngularVelocity( node_id);
    // TODO add noise?
    _calibrated_sensors.write( imusens );

}

