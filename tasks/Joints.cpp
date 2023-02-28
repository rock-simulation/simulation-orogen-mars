/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Joints.hpp"
#include <boost/foreach.hpp>
#include <mars_interfaces/sim/MotorManagerInterface.h>
#include <mars_interfaces/sim/ControlCenter.h>
#include <base/Logging.hpp>
#include <base/samples/RigidBodyState.hpp>


using namespace mars;

// TODO: we should get it from simulator
#define SIM_CENTER_FRAME_NAME "world"

// TODO: add passive joint, without motors
// TODO: check effort, it seems to be always zero

Joints::Joints(std::string const& name)
    : JointsBase(name)
{
}

Joints::Joints(std::string const& name, RTT::ExecutionEngine* engine)
    : JointsBase(name, engine)
{
    //controlMode = mars::IGNORE;
}

Joints::~Joints()
{
}

bool Joints::configureHook()
{
    if (!JointsBase::configureHook())
        return false;

    // set robot name as prefix
    if (_robot_name.value() != "")
        prefix = _robot_name;

    // add prefix to the joint names from config to be able to find the joints inside the mars
    // since all mars elements contains prefix in their names
    jointNames.clear();
    jointNames = _names.value();
    std::for_each(jointNames.begin(), jointNames.end(),
        [&](std::string &jointName)
        { jointName = prefix + jointName; });

    // TODO: for now we get SubWorld frame by its frame name, it can be changed later
    // find all joints that required by config
    const VertexDesc subWorldVertex = control->envireGraph->vertex("SubWorld::" + prefix);
    findJoints(subWorldVertex);

    // initialise jointStatus vector with the name of required joints
    size_t num_joints = jointNames.size();
    jointStatus.resize(num_joints);
    jointStatus.names = jointNames;

    return true;
}
bool Joints::startHook()
{
    if (!JointsBase::startHook())
        return false;
    return true;
}
void Joints::updateHook()
{
    JointsBase::updateHook();
}
void Joints::errorHook()
{
    JointsBase::errorHook();
}
void Joints::stopHook()
{
    JointsBase::stopHook();
}
void Joints::cleanupHook()
{
    motorJoints.clear();
    passiveJoint.clear();
    prefix = "";
    jointNames.clear();

    JointsBase::cleanupHook();
}

void Joints::update(double time)
{
    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    while (_command.read(jointCommand) == RTT::NewData)
    {
        for (auto &jointName : jointCommand.names)
        {
            if (motorJoints.count(jointName)) {
                base::JointState state = jointCommand.getElementByName(jointName);

                if( state.hasPosition() )
                {
                    motorJoints[jointName].first->setValue(state.position);
                }
                else if( state.hasSpeed() ) {
                    motorJoints[jointName].first->setVelocity(state.speed);
                }
                if( state.hasEffort() )
                {
                    LOG_WARN_S << "Effort command ignored for the joint '" << jointName << "'";
                }
                if( state.hasRaw() )
                {
                    LOG_WARN_S << "Raw command ignored for the joint '" << jointName << "'";
                }
            } else {
                LOG_ERROR_S << "There is no joint with the name '" << jointName << "' or this joint is passive";
            }

        }
    }

    // send the current state of the joints
    for (auto &jointName : jointStatus.names)
    {
        base::JointState state;
        // check if the joint has motor or passive
        // get the data from mars
        if (motorJoints.count(jointName)) {
            MJPair &mj = motorJoints[jointName];
            state.position = mj.first->getActualPosition();
            state.speed = mj.second->getVelocity();
            // TODO: should we use getMotorTorque?
            //state.effort = mj.first->getTorque();
            state.effort = mj.second->getMotorTorque();

            jointStatus[jointName] = state;
        }
        else {
            // TODO add passive joint
        }
    }
    jointStatus.time = getTime();
    _status.write(jointStatus);
}

void Joints::findJoints(const VertexDesc &vertex)
{
    // parse the sub graph to find all required joints and their corresponding motors
    if(control->graphTreeView->tree.find(vertex) != control->graphTreeView->tree.end())
    {
        const std::unordered_set<VertexDesc>& children = control->graphTreeView->tree[vertex].children;
        // TODO: there is some issue if children is empty
        if (!children.empty()) {
            for(const VertexDesc child : children)
            {
                using JointItem = envire::core::Item<interfaces::JointInterfaceItem>;
                using JointItemItr = envire::core::EnvireGraph::ItemIterator<JointItem>;
                JointItemItr begin_joint, end_joint, cur_joint;
                boost::tie(begin_joint, end_joint) = control->envireGraph->getItems<JointItem>(child);

                using SimMotorItem = envire::core::Item<std::shared_ptr<core::SimMotor>>;
                using SimMotorItemItr = envire::core::EnvireGraph::ItemIterator<SimMotorItem>;
                SimMotorItemItr begin_motor, end_motor, cur_motor;
                boost::tie(begin_motor, end_motor) = control->envireGraph->getItems<SimMotorItem>(child);

                // parse the joints of the frame to find the joints required by config
                for(cur_joint = begin_joint; cur_joint!=end_joint; cur_joint++)
                {
                    // all joint names saved with the prefix (robot name) inside the mars
                    std::shared_ptr<interfaces::JointInterface> jointInterface = cur_joint->getData().jointInterface;
                    std::string jointName;
                    jointInterface->getName(&jointName);

                    // check if the joint is required by config
                    if (std::find(jointNames.begin(), jointNames.end(), jointName) != std::end(jointNames))
                    {
                        // check if there is a motor for joint, than store the motor joint relation
                        // if not, store joint as passive
                        bool hasMotor = false;
                        cur_motor = begin_motor;
                        while (cur_motor != end_motor && hasMotor == false) {
                            std::shared_ptr<core::SimMotor> simMotor = cur_motor->getData();
                            // TODO: this is a temporary fix to match names of joints and joint names in the simMotor
                            // since later joint and motor are stored in their own frame
                            if (std::string(prefix + simMotor->getJointName()) == jointName)
                            {
                                motorJoints[simMotor->getName()] = std::make_pair(simMotor, jointInterface);
                                hasMotor = true;
                            }
                            cur_motor++;
                        }

                        if (hasMotor == false)
                        {
                            passiveJoint.push_back(jointInterface);
                        }
                    }
                }
                findJoints(child);
            }
        }
    } else {
        LOG_ERROR_S << "SubWorld " << "SubWorld::" << prefix << " can not be found";
    }
}