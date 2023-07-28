/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Tether.hpp"
#include <base-logging/Logging.hpp>
#include <mars/plugins/tether_simulation/TetherSimulation.h>

using namespace mars;

Tether::Tether(std::string const& name)
    : TetherBase(name)
{
    targetSpeed.store(0);
    pluginInitialized = false;
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

    launch_tether_plugin = _launch_tether_plugin.get();

    return true;
}
bool Tether::startHook()
{
    if (! TetherBase::startHook())
        return false;

    if(launch_tether_plugin)
    {
    LOG_DEBUG("Beginning of the startHook: About to get the tether plugin from Mars");
    mars::interfaces::MarsPluginTemplate *TetherPluginInterface;
    if (Task::getPlugin("tether_simulation", TetherPluginInterface))
    {
        tether_plugin = dynamic_cast<mars::plugins::tether_simulation::TetherSimulation*>(TetherPluginInterface);
        
        LOG_DEBUG("Plugin name : %s \n",tether_plugin->getLibName().c_str());
    }
    else
    {
        LOG_DEBUG("Failed to obtain the Tether management plugin\n");
    }
    }

    return true;
}
void Tether::updateHook()
{
    TetherBase::updateHook();
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);

    if(launch_tether_plugin)
    {
    base::samples::Joints cmd;

    while (_winch_command.read(cmd) == RTT::NewData ) {
        targetSpeed.store(cmd["winch"].speed);
        tether_plugin->setSpeed((double)targetSpeed.load());
    }

    float newspeed = 0;
    while (_winch_speed.read(newspeed) == RTT::NewData ) {
        targetSpeed.store(newspeed);
        tether_plugin->setSpeed((double)newspeed);
    }

    _rope_length.write(tether_plugin->getFixedLength());
    _winch_force.write(tether_plugin->getForces());
    }
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

void  Tether::update( double time )
{
    if(launch_tether_plugin)
    {

    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    if(!pluginInitialized)
    {
        tether_plugin->updateSettings(_tether_settings.value());
        tether_plugin->init();
        pluginInitialized = true;
    }
    }
}