/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TMDS_Simulation.hpp"
#include <lib_manager/LibManager.hpp>
#include <mars/plugins/tether_simulation/TetherSimulation.h>

using namespace mars;

TMDS_Simulation::TMDS_Simulation(std::string const& name)
    : TMDS_SimulationBase(name)
{
}

TMDS_Simulation::~TMDS_Simulation()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TMDS_Simulation.hpp for more detailed
// documentation about them.

bool TMDS_Simulation::configureHook()
{
    if (! TMDS_SimulationBase::configureHook())
        return false;
    return true;
}
bool TMDS_Simulation::startHook()
{
    if (! TMDS_SimulationBase::startHook())
        return false;
    
    mars::interfaces::SimulatorInterface *simulation = Task::getSimulatorInterface();
        if( !simulation ){
            std::cerr << "Plugin: could not get singleton instance of simulator interface." << std::endl;
            RTT::log(RTT::Error) << "Plugin: could not get singleton instance of simulator interface." << std::endl;
            return false;
        }

    mars::interfaces::ControlCenter *control = simulation->getControlCenter();

    lib_manager::LibManager *libManager = control->getLibManager();
    mars::interfaces::MarsPluginTemplate *tetherInterface = libManager.getLibraryAs<mars::interfaces::MarsPluginTemplate>("tether_simulation"); 
    if (tetherInterface) 
    {
    // tetherPlugin= dynamic_cast<tether_simulation::TetherSimulation*>(tetherInterface);
    std::cout << "Plugin found!" << std::endl;
  
    }

    return true;
}
void TMDS_Simulation::updateHook()
{
    TMDS_SimulationBase::updateHook();
}
void TMDS_Simulation::errorHook()
{
    TMDS_SimulationBase::errorHook();
}
void TMDS_Simulation::stopHook()
{
    TMDS_SimulationBase::stopHook();
}
void TMDS_Simulation::cleanupHook()
{
    TMDS_SimulationBase::cleanupHook();
}
