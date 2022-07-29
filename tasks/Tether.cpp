/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Tether.hpp"

#include <base-logging/Logging.hpp>
using namespace mars;

Tether::Tether(std::string const& name)
    : TetherBase(name)
{
}

Tether::~Tether()
{
}

// Operations
void Tether::dock()
{
    mars::interfaces::NodeId robot_node_id = getNodeID(_robot_link.get());
    mars::interfaces::NodeId tether_node_id = getNodeID(_tether_link.get());
    if ( tether_node_id != 0 &&
         robot_node_id  != 0 )
    {
        control->sim->connectNodes(robot_node_id, tether_node_id);
        LOG_INFO_S << "Successfully docked";
    }
}

void Tether::undock()
{
    mars::interfaces::NodeId robot_node_id = getNodeID(_robot_link.get());
    mars::interfaces::NodeId tether_node_id = getNodeID(_tether_link.get());
    if ( tether_node_id != 0 &&
         robot_node_id  != 0 )
    {
        control->sim->disconnectNodes(robot_node_id, tether_node_id);
        LOG_INFO_S << "Successfully undocked";
    }
}

mars::interfaces::NodeId Tether::getNodeID(const std::string & link)
{
    if (link == "")
    {
        LOG_ERROR_S << "Properties robot_link and tether_link must not be empty.";
        return 0;
    }

    mars::interfaces::NodeId nodeID = control->nodes->getID(link);
    if ( nodeID == 0 )
    {
        LOG_ERROR_S << "Could not determine nodeID for " << link;
        return 0;
    } else {
        return nodeID;
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Tether.hpp for more detailed
// documentation about them.

bool Tether::configureHook()
{
    if (! TetherBase::configureHook())
        return false;
    return true;
}
bool Tether::startHook()
{
    if (! TetherBase::startHook())
        return false;
    

    mars::interfaces::MarsPluginTemplate *TetherPluginInterface;
    if (Task::getPlugin("tether_simulation", TetherPluginInterface))
    {
        LOG_DEBUG("The Tether management plugin from the simulation has been obtained");
    }
    else
    {
        LOG_DEBUG("Failed to obtain the Tether management plugin");
    }

    return true;
}
void Tether::updateHook()
{
    TetherBase::updateHook();
}
void Tether::errorHook()
{
    TetherBase::errorHook();
}
void Tether::stopHook()
{
    TetherBase::stopHook();
}
void Tether::cleanupHook()
{
    TetherBase::cleanupHook();
}
