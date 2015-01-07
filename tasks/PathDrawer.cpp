/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PathDrawer.hpp"
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <base/Logging.hpp>

using namespace mars;

PathDrawer::PathDrawer(std::string const& name)
    : PathDrawerBase(name)
{
}

PathDrawer::PathDrawer(std::string const& name, RTT::ExecutionEngine* engine)
    : PathDrawerBase(name, engine)
{
}

PathDrawer::~PathDrawer()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PathDrawer.hpp for more detailed
// documentation about them.
bool PathDrawer::configureHook()
{
    if (! mars::Plugin::configureHook())
        return false;
    
    // create the line handle
    osg_lines::LinesFactory lF;
    l = lF.createLines();

    return true;
}

bool PathDrawer::startHook()
{
    if (! mars::Plugin::startHook())
        return false;
    
    return true;
}

void PathDrawer::updateHook()
{
    mars::Plugin::updateHook();

    // read the current trajectory
    std::vector<base::Trajectory> trajectories_2d, trajectories_3d;
    if(_trajectories_2d.readNewest(trajectories_2d) != RTT::NoData){
        // clear old path
        control->graphics->removeOSGNode(l->getOSGNode());
        osg_lines::LinesFactory lF;
        l = lF.createLines();

        // get each xy coordinate
        std::vector<base::Trajectory>::iterator it;
        for(it = trajectories_2d.begin(); it != trajectories_2d.end(); ++it){

            // get the dimension
            int dim = it->spline.getDimension();
            if(dim < 2 || dim > 3){
                LOG_WARN_S << "2d or 3d (where z-component will be neglected) trajectory needed";
                continue;
            }

            // get the coordinates and generate 3d coordinates
            std::vector<double> path = it->spline.getCoordinates();
            base::Vector3d v;
            std::vector<base::geometry::SplineBase::CoordinateType> coord_types;
            std::vector<base::Vector3d> waypoints;
            std::vector<double>::iterator val;
            for(val = path.begin(); val != path.end(); val += dim){
                v[0] = *val;
                v[1] = *(val+1);
                v[2] = getHeightFromScene(v[0], v[1]) + _distance_to_ground.get();
                //printf("adding point %g / %g\n", v[0], v[1]);
                l->appendData(osg_lines::Vector(v[0], v[1], v[2]));

                // write the z-coridnate in the trajectory
                coord_types.push_back(base::geometry::SplineBase::ORDINARY_POINT);
                waypoints.push_back(v);
            }

            // use the 3d points to generate the 3d trajectory
            std::vector<double> parameters;
            base::Trajectory new_trajectory;
            try {
                new_trajectory.spline.interpolate(waypoints, parameters, coord_types);
            } catch (std::runtime_error& e) {
                LOG_ERROR_S << "Spline exception: " << e.what();
            }
            trajectories_3d.push_back(new_trajectory);
        }

        // draw it
        l->setColor(osg_lines::Color(0.0, 1.0, 0.0, 1.0));
        l->setLineWidth(4);
        control->graphics->addOSGNode(l->getOSGNode());

        // send the 3d coordinate
        _trajectories_3d.write(trajectories_3d);
    }
}

void PathDrawer::errorHook()
{
    mars::Plugin::errorHook();
}


void PathDrawer::stopHook()
{
    mars::Plugin::stopHook();
}


void PathDrawer::cleanupHook()
{
    mars::Plugin::cleanupHook();
}


mars::interfaces::sReal PathDrawer::getHeightFromScene(mars::interfaces::sReal x, mars::interfaces::sReal y){
    mars::interfaces::PhysicsInterface* physics = control->sim->getPhysics();
    mars::interfaces::sReal z = 10.0;
    const utils::Vector ray_origin(x, y, z);
    const utils::Vector ray_vector(0.0, 0.0, -20);
    mars::interfaces::sReal value = z - physics->getVectorCollision(ray_origin, ray_vector);

    return value;
}
