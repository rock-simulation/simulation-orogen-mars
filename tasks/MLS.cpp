/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MLS.hpp"

#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/StorageManagerInterface.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <maps/grid/MLSMap.hpp>


using namespace mars;

using MLSPrecalculatedItem = envire::core::Item<maps::grid::MLSMap<maps::grid::MLSConfig::PRECALCULATED>>;
using MLSPrecalculatedItr = envire::core::EnvireGraph::ItemIterator<MLSPrecalculatedItem>;


MLS::MLS(std::string const& name)
    : MLSBase(name)
{
}

MLS::~MLS()
{
}

bool MLS::setupMLSPrecSimulation(::base::samples::RigidBodyState const & robotPose, ::maps::grid::MLSMap< ::maps::grid::MLSConfig::PRECALCULATED > const & mls)
{
    std::string mls_frame = "mls_01";
    // check if the mls was set before, delete it
    // we will have one MLS per frame
    if (sim->getControlCenter()->storage->getGraph()->containsFrame(mls_frame)) {

        MLSPrecalculatedItr itr, end_itr;
        boost::tie(itr, end_itr) = control->storage->getGraph()->getItems<MLSPrecalculatedItem>(mls_frame);
        for (;itr!=end_itr; itr++)
        {
            sim->getControlCenter()->storage->getGraph()->removeItemFromFrame(mls_frame, itr);
        }
    } else {
        LOG_ERROR("[MLS::setupMLSPrecSimulation] the graph does not contain frame " + mls_frame);
        return false;
    }

    envire::core::Item<maps::grid::MLSMap<maps::grid::MLSConfig::PRECALCULATED>>::Ptr mlsItemPtr(new envire::core::Item<maps::grid::MLSMap<maps::grid::MLSConfig::PRECALCULATED>>(mls));
    sim->getControlCenter()->storage->getGraph()->addItemToFrame("mls_01", mlsItemPtr);
    LOG_DEBUG("[MLS::setupMLSPrecSimulation] MLS added");
    return true;
}

bool MLS::setupMLSSlopeSimulation(::base::samples::RigidBodyState const & robotPose, ::maps::grid::MLSMap< ::maps::grid::MLSConfig::SLOPE > const & mls)
{
    // check if the mls was set before
    // and replace it by new one

    maps::grid::MLSMap<maps::grid::MLSConfig::PRECALCULATED> mls_pre = mls;

    return setupMLSPrecSimulation(robotPose, mls_pre);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MLS.hpp for more detailed
// documentation about them.

bool MLS::configureHook()
{
    if (! MLSBase::configureHook())
        return false;
    return true;
}
bool MLS::startHook()
{
    if (! MLSBase::startHook())
        return false;
    return true;
}
void MLS::updateHook()
{
    MLSBase::updateHook();

    maps::grid::MLSMap< ::maps::grid::MLSConfig::PRECALCULATED > mls_pre;
    if( _mls_prec.read( mls_pre ) == RTT::NewData )
    {
        bool success = setupMLSPrecSimulation(base::samples::RigidBodyState(), mls_pre);
        _is_mls_set.write(success);
    }

    maps::grid::MLSMap< ::maps::grid::MLSConfig::SLOPE > mls_slope;
    if( _mls_slope.read( mls_slope ) == RTT::NewData )
    {
        bool success = setupMLSSlopeSimulation(base::samples::RigidBodyState(), mls_slope);
        _is_mls_set.write(success);
    }
}
void MLS::errorHook()
{
    MLSBase::errorHook();
}
void MLS::stopHook()
{
    MLSBase::stopHook();
}
void MLS::cleanupHook()
{
    MLSBase::cleanupHook();
}
