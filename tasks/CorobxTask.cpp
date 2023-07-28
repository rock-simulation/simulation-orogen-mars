/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CorobxTask.hpp"
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/StorageManagerInterface.h>

#include <base-logging/Logging.hpp>

using namespace mars;

CorobxTask::CorobxTask(std::string const& name)
    : CorobxTaskBase(name)
{
}

CorobxTask::~CorobxTask()
{
}


bool CorobxTask::set_pose(::base::samples::RigidBodyState const & pose) {
    LOG_DEBUG_S << "Setting up initial position to " << pose.position[0] << ", " << pose.position[1] << ", " << pose.position[2];
    LOG_DEBUG_S << "Setting up initial orientation to x: " << pose.orientation.x() << ", y:" << pose.orientation.y() << ", z:" << pose.orientation.z() << ", w:"<<pose.orientation.w();
    LOG_DEBUG_S << "Setting up target frame to " << pose.targetFrame;
    bool success = control->nodes->setAbsolutePose(pose.targetFrame, pose.position, pose.orientation);
    LOG_DEBUG("Position given to the node manager");
    return success;
}
/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CorobxTask.hpp for more detailed
// documentation about them.

bool CorobxTask::configureHook()
{
    if (! CorobxTaskBase::configureHook())
        return false;
    return true;
}
bool CorobxTask::startHook()
{
    if (! CorobxTaskBase::startHook())
        return false;
    return true;
}
void CorobxTask::updateHook()
{
    CorobxTaskBase::updateHook();


    base::samples::RigidBodyState pose;
    if (_pose.readNewest(pose)  == RTT::NewData)
    {
        // TODO: only absolute position is implemented for now
        bool success = set_pose(pose);
        _is_pose_set.write(success);
    }

    bool run_simulation = false;
    if(_run_simulation.read(run_simulation) == RTT::NewData)
    {
        if (run_simulation) {
            if (!control->sim->isSimRunning()) {
                control->sim->StartSimulation();
            }
        } else {
            if (control->sim->isSimRunning())
            {
                control->sim->StopSimulation();
                // wait until simulation is stopped
                // since it will go first in stopping state
                // before it stoped
                // we hope it will stop anyway :)
                //while(simulatorInterface->isSimRunning())
                //    msleep(10);
            }
        }
    }

    if (control->sim->isSimRunning()) {
        mars::utils::Vector position;
        mars::utils::Quaternion orientation;
        LOG_DEBUG_S << "Getting absolute pose";
        bool success = control->nodes->getAbsolutePose(_frame_for_current_pose.get(), position, orientation);
        if (success)
        {
            LOG_DEBUG_S << "Absolute pose successfully received";
            base::samples::RigidBodyState current_pose;
            current_pose.time = base::Time::now();
            current_pose.sourceFrame = control->storage->getRootFrame();
            LOG_DEBUG_S << "Current pose root frame" << current_pose.sourceFrame;
            current_pose.targetFrame = _frame_for_current_pose.get();
            LOG_DEBUG_S << "Current pose target frame" << current_pose.targetFrame;
            current_pose.position = position;
            LOG_DEBUG_S << "Current pose position" << current_pose.position[0] << ", " << current_pose.position[1] << ", " << current_pose.position[2];
            current_pose.orientation = orientation;
            LOG_DEBUG_S << "Current pose orientation <x,y,z,w>" << current_pose.orientation.x() << ", " << current_pose.orientation.y() << ", " << current_pose.orientation.z()<< ", " << current_pose.orientation.w();
            _current_pose.write(current_pose);
            LOG_DEBUG_S << "Current pose sent out";
        }
    }
}
void CorobxTask::errorHook()
{
    CorobxTaskBase::errorHook();
}
void CorobxTask::stopHook()
{
    CorobxTaskBase::stopHook();
}
void CorobxTask::cleanupHook()
{
    CorobxTaskBase::cleanupHook();
}
