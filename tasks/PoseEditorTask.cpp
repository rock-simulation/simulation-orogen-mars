/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseEditorTask.hpp"
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

using namespace mars;

PoseEditorTask::PoseEditorTask(std::string const& name)
    : PoseEditorTaskBase(name)
{
}

PoseEditorTask::~PoseEditorTask()
{
}


bool PoseEditorTask::set_pose(::base::samples::RigidBodyState const & pose) {
    bool success = control->nodes->setAbsolutePose(pose.targetFrame, pose.position, pose.orientation);
    return success;
}
/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseEditorTask.hpp for more detailed
// documentation about them.

bool PoseEditorTask::configureHook()
{
    if (! PoseEditorTaskBase::configureHook())
        return false;
    return true;
}
bool PoseEditorTask::startHook()
{
    if (! PoseEditorTaskBase::startHook())
        return false;
    return true;
}
void PoseEditorTask::updateHook()
{
    PoseEditorTaskBase::updateHook();


    base::samples::RigidBodyState pose;
    if (_pose.readNewest(pose))
    {
        // TODO: only absolute position is implemented for now
        bool success = set_pose(pose);
        _is_pose_set.write(success);
    }
}
void PoseEditorTask::errorHook()
{
    PoseEditorTaskBase::errorHook();
}
void PoseEditorTask::stopHook()
{
    PoseEditorTaskBase::stopHook();
}
void PoseEditorTask::cleanupHook()
{
    PoseEditorTaskBase::cleanupHook();
}
