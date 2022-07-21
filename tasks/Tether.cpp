/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Tether.hpp"
#include <lib_manager/LibManager.hpp>
#include <mars/plugins/tether_simulation/TetherSimulation.h>

#include <base-logging/Logging.hpp>
using namespace mars;

Tether::Tether(std::string const& name)
    : TetherBase(name)
{
}

Tether::~Tether()
{
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


    // mars::interfaces::SimulatorInterface *simulation = Task::getSimulatorInterface();
    //     if( !simulation ){
    //         LOG_ERROR("Tether: could not get singleton instance of simulator interface.");
    //         RTT::log(RTT::Error) << "Plugin: could not get singleton instance of simulator interface." << std::endl;
    //         return false;
    //     }

    // From the controlCenter we can access the simulatorInterface (we already have it )
    //and the LoadCenter none of them give access to the libraries
    //mars::interfaces::ControlCenter *control = simulation->getControlCenter();
    //Tasks::libManager ->
    //lib_manager::LibManager *libManager = control->getLibManager();
    //mars::interfaces::MarsPluginTemplate *TetherInterface = libManager.getLibraryAs<mars::interfaces::MarsPluginTemplate>("Tether_simulation"); 
    //if (TetherInterface) 
    //{
    //// TetherPlugin= dynamic_cast<Tether_simulation::TetherSimulation*>(TetherInterface);
    //std::cout << "Plugin found!" << std::endl;
  
    //}

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
