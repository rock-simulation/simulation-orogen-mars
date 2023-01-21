/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CorobxTask.hpp"
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/StorageManagerInterface.h>

using namespace mars;

CorobxTask::CorobxTask(std::string const& name)
    : CorobxTaskBase(name)
{
}

CorobxTask::~CorobxTask()
{
}


bool CorobxTask::set_pose(::base::samples::RigidBodyState const & pose) {
    bool success = control->nodes->setAbsolutePose(pose.targetFrame, pose.position, pose.orientation);
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
        std::cout << "test 1" << std::endl;
        bool success = control->nodes->getAbsolutePose(_frame_for_current_pose.get(), position, orientation);
        std::cout << "test 2" << std::endl;
        if (success)
        {
            std::cout << "test 3" << std::endl;
            base::samples::RigidBodyState current_pose;
            current_pose.time = base::Time::now();
            std::cout << "test 4" << std::endl;
            current_pose.sourceFrame = control->storage->getRootFrame();
            std::cout << "test 5" << std::endl;
            current_pose.targetFrame = _frame_for_current_pose.get();
            std::cout << "test 6" << std::endl;
            current_pose.position = position;
            std::cout << "test 7" << std::endl;
            current_pose.orientation = orientation;
            std::cout << "test 8" << std::endl;
            _current_pose.write(current_pose);
            std::cout << "test 9" << std::endl;
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
