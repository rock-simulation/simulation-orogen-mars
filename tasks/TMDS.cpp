/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TMDS.hpp"
#include <lib_manager/LibManager.hpp>
#include <mars/plugins/tether_simulation/TetherSimulation.h>

#include <base-logging/Logging.hpp>
using namespace mars;

TMDS::TMDS(std::string const& name)
    : TMDSBase(name)
{
}

TMDS::~TMDS()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TMDS.hpp for more detailed
// documentation about them.

bool TMDS::configureHook()
{
    if (! TMDSBase::configureHook())
        return false;
    return true;
}
bool TMDS::startHook()
{
    if (! TMDSBase::startHook())
        return false;
    

    mars::interfaces::MarsPluginTemplate *tetherPluginInterface;
    if (Task::getPlugin("tether_simulation", tetherPluginInterface))
    {
        LOG_DEBUG("The tether management plugin from the simulation has been obtained");
    }
    else
    {
        LOG_DEBUG("Failed to obtain the tether management plugin");
    }


    // mars::interfaces::SimulatorInterface *simulation = Task::getSimulatorInterface();
    //     if( !simulation ){
    //         LOG_ERROR("TMDS: could not get singleton instance of simulator interface.");
    //         RTT::log(RTT::Error) << "Plugin: could not get singleton instance of simulator interface." << std::endl;
    //         return false;
    //     }

    // From the controlCenter we can access the simulatorInterface (we already have it )
    //and the LoadCenter none of them give access to the libraries
    //mars::interfaces::ControlCenter *control = simulation->getControlCenter();
    //Tasks::libManager ->
    //lib_manager::LibManager *libManager = control->getLibManager();
    //mars::interfaces::MarsPluginTemplate *tetherInterface = libManager.getLibraryAs<mars::interfaces::MarsPluginTemplate>("tether_simulation"); 
    //if (tetherInterface) 
    //{
    //// tetherPlugin= dynamic_cast<tether_simulation::TetherSimulation*>(tetherInterface);
    //std::cout << "Plugin found!" << std::endl;
  
    //}

    return true;
}
void TMDS::updateHook()
{
    TMDSBase::updateHook();
}
void TMDS::errorHook()
{
    TMDSBase::errorHook();
}
void TMDS::stopHook()
{
    TMDSBase::stopHook();
}
void TMDS::cleanupHook()
{
    TMDSBase::cleanupHook();
}
