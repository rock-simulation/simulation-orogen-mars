/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "HighResRangeFinder.hpp"
#include <mars/interfaces/sim/SensorManagerInterface.h>

using namespace mars;

HighResRangeFinder::HighResRangeFinder(std::string const& name)
    : HighResRangeFinderBase(name)
{
}

HighResRangeFinder::HighResRangeFinder(std::string const& name, RTT::ExecutionEngine* engine)
    : HighResRangeFinderBase(name, engine)
{
}

HighResRangeFinder::~HighResRangeFinder()
{
    std::vector<Camera*>::iterator it = cameras.begin();
    for(; it != cameras.end(); ++it) {
        delete *it;
    }
    cameras.clear();
}

bool HighResRangeFinder::addCamera(::std::string const & name, double orientation)
{
    long sensor_id = control->sensors->getSensorID( name );
    if( !sensor_id ){
	    RTT::log(RTT::Error) << "There is no camera by the name of " << name << " in the scene" << RTT::endlog();
        return false;
    }
    
    mars::sim::CameraSensor* cam_sensor = dynamic_cast<mars::sim::CameraSensor *>(control->sensors->getSimSensor(sensor_id));
    if( !cam_sensor){
        RTT::log(RTT::Error) << "CameraPlugin: Given sensor name is not a camera" << RTT::endlog();
        return false;
    }
    
    HighResRangeFinderCamera cam = { name, orientation };
    addedCameras.push_back(cam);

    //using !cameras.empty() as a proxy for the task being started. In a started task,
    //there will always be at least one camera.
    if(!cameras.empty()) {
        Camera* camera = new Camera(sensor_id, cam_sensor, orientation);
        camera->name = name;
        calcCamParameters(camera);
        cameras.push_back(camera);
    }
    return true;
}
    
    
void HighResRangeFinder::calcCamParameters(Camera* camera) {
    
    int width = camera->cam_sensor_info.width;
    int height = camera->cam_sensor_info.height;
    double opening_width = camera->cam_sensor_info.opening_width;
    double opening_height = camera->cam_sensor_info.opening_height;

    // These conversions are not entirely correct since the image pixels
    // are linear on a straight line, while angles are linear on a circumference.
    // see below for the correct calculations.

    // Calculate starting pixel (bottom left within the image plane)
    double pixel_per_rad_horizontal = width / ((opening_width / 180.0) * M_PI);
    double pixel_per_rad_vertical = height / ((opening_height / 180.0) * M_PI);

    double pixel_per_tan_rad_horizontal = width / 2.0 / tan( opening_width / 180.0 * M_PI / 2.0 );
    double pixel_per_tan_rad_vertical = height / 2.0 / tan( opening_height / 180.0 * M_PI / 2.0 );

    RTT::log(RTT::Info) << "Camera " << camera->sensor_id << " (" << camera->name << ") added" << RTT::endlog();
    RTT::log(RTT::Info) << "opening_width " << opening_width << ", opening_height " << opening_height << RTT::endlog();

    camera->points.clear();

    if(!_rotational_horizontal.get()) {
        // Sets the borders.
        double lower_pixel = _lower_limit.get() * pixel_per_rad_vertical + height/2.0;
        double upper_pixel = _upper_limit.get() * pixel_per_rad_vertical + height/2.0;
        double left_pixel = _left_limit.get() * pixel_per_rad_horizontal + width/2.0;
        double right_pixel = _right_limit.get() * pixel_per_rad_horizontal + width/2.0;

        if(lower_pixel < 0) {
            RTT::log(RTT::Warning) << "Lower limit exceeds the image plane, will be scaled down by " <<
                    std::fabs(lower_pixel) << " pixel" << RTT::endlog();
            lower_pixel = 0;
        }
        if(upper_pixel > height) {
            RTT::log(RTT::Warning) << "Upper limit exceeds the image plane, will be scaled down by " <<
                    upper_pixel - height << " pixel" << RTT::endlog();
            upper_pixel = height;
        }
        if(left_pixel < 0) {
            RTT::log(RTT::Warning) << "Left limit exceeds the image plane, will be scaled down by " <<
                    std::fabs(left_pixel) << " pixel" << RTT::endlog();
            left_pixel = 0;
        }
        if(right_pixel > width) {
            RTT::log(RTT::Warning) << "Right limit exceeds the image plane, will be scaled down by " <<
                    right_pixel - width << " pixel" << RTT::endlog();
            right_pixel = width;
        }

        double v_steps = _resolution_vertical.get() * pixel_per_rad_vertical;
        double h_steps = _resolution_horizontal.get() * pixel_per_rad_horizontal;

        for(double y = lower_pixel; y < upper_pixel; y += v_steps) {
            for(double x = left_pixel; x < right_pixel; x += h_steps) {
                // Pixel contains a distance value between min and max
                // and lies within the image plane.
                LookupPoint pt = {(size_t) x, (size_t) y};
                camera->points.push_back(pt);
            }
        }

        RTT::log(RTT::Info) << "Horizontal: Every " << h_steps << " pixel " << " will be used from " <<
                left_pixel << " to " << right_pixel << RTT::endlog();
        RTT::log(RTT::Info) << "Vertical: Every " << v_steps << " pixel " << " will be used from " <<
                lower_pixel << " to " << upper_pixel << RTT::endlog();
    } else {
        //this is basically a vertical cylinder; the sensor could be a rotating line-ccd

        // Sets the borders.
        double lower_pixel = tan(_lower_limit.get()) * pixel_per_tan_rad_vertical + height/2.0;
        double upper_pixel = tan(_upper_limit.get()) * pixel_per_tan_rad_vertical + height/2.0;

        if(lower_pixel < 0) {
            RTT::log(RTT::Warning) << "Lower limit exceeds the image plane, will be scaled down by " <<
                    std::fabs(lower_pixel) << " pixel" << RTT::endlog();
            lower_pixel = 0;
        }
        if(upper_pixel > height) {
            RTT::log(RTT::Warning) << "Upper limit exceeds the image plane, will be scaled down by " <<
                    upper_pixel - height << " pixel" << RTT::endlog();
            upper_pixel = height;
        }

        double v_steps = _resolution_vertical.get() * pixel_per_rad_vertical;

        for(double y = lower_pixel; y < upper_pixel; y += v_steps) {
            for(double x_rad = _left_limit.get();
                       x_rad < _right_limit.get();
                       x_rad += _resolution_horizontal.get()) {
                LookupPoint pt = {
                    (size_t) (tan(x_rad) * pixel_per_tan_rad_horizontal + width/2.0 ),
                    (size_t) ((y-height/2.0)/cos(x_rad) + height/2.0) };
                if(pt.x >= 0 && pt.y >= 0 &&
                    pt.x < width && pt.y < height) {
                    bool found = false;
                    for(auto &p : camera->points) {
                        if(p.x == pt.x && p.y == pt.y) {
                            found = true;
                            break;
                        }
                    }
                    if(!found)
                        camera->points.push_back(pt);
                }
            }
        }

        RTT::log(RTT::Info) << "Horizontal: Every " << _resolution_horizontal.get() << " rad " << " will be used from " <<
                _left_limit.get() << " to " << _right_limit.get() << RTT::endlog();
        RTT::log(RTT::Info) << "Vertical: Every " << v_steps << " pixel " << " will be used from " <<
                lower_pixel << " to " << upper_pixel << RTT::endlog();
    }

}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See HighResRangeFinder.hpp for more detailed
// documentation about them.

bool HighResRangeFinder::configureHook()
{
    if (! HighResRangeFinderBase::configureHook())
        return false;
        
    return true;
}
bool HighResRangeFinder::startHook()
{
    if (! HighResRangeFinderBase::startHook())
        return false;
        
    // Adds this camera to the list of cameras.
    Camera* camera = new Camera(sensor_id, this->camera, 0.0);
    calcCamParameters(camera);
    cameras.push_back(camera);

    for(auto &c : addedCameras) {
        long sensor_id = control->sensors->getSensorID( c.name );
        if( !sensor_id ){
            RTT::log(RTT::Error) << "There is no camera by the name of " << c.name << " in the scene" << RTT::endlog();
        }

        mars::sim::CameraSensor* cam_sensor = dynamic_cast<mars::sim::CameraSensor *>(control->sensors->getSimSensor(sensor_id));
        if( !cam_sensor){
            RTT::log(RTT::Error) << "CameraPlugin: Given sensor name is not a camera" << RTT::endlog();
        }

        Camera* camera = new Camera(sensor_id, cam_sensor, c.orientation);
        camera->name = c.name;
        calcCamParameters(camera);
        cameras.push_back(camera);
    }

    for(auto &c : _extra_cameras.get()) {
        long sensor_id = control->sensors->getSensorID( c.name );
        if( !sensor_id ){
            RTT::log(RTT::Error) << "There is no camera by the name of " << c.name << " in the scene" << RTT::endlog();
        }

        mars::sim::CameraSensor* cam_sensor = dynamic_cast<mars::sim::CameraSensor *>(control->sensors->getSimSensor(sensor_id));
        if( !cam_sensor){
            RTT::log(RTT::Error) << "CameraPlugin: Given sensor name is not a camera" << RTT::endlog();
        }

        Camera* camera = new Camera(sensor_id, cam_sensor, c.orientation);
        camera->name = c.name;
        calcCamParameters(camera);
        cameras.push_back(camera);
    }

    return true;
}

void HighResRangeFinder::updateHook()
{
    HighResRangeFinderBase::updateHook();
}
void HighResRangeFinder::errorHook()
{
    HighResRangeFinderBase::errorHook();
}
void HighResRangeFinder::stopHook()
{
    HighResRangeFinderBase::stopHook();

    std::vector<Camera*>::iterator it = cameras.begin();
    for(; it != cameras.end(); ++it) {
        delete *it;
    }
    cameras.clear();
}
void HighResRangeFinder::cleanupHook()
{
    HighResRangeFinderBase::cleanupHook();
}

void HighResRangeFinder::getData()
{	
    base::samples::Pointcloud pointcloud;
    pointcloud.time = getTime();
    std::vector<Camera*>::iterator it = cameras.begin();
    for(; it < cameras.end(); ++it) {
        int counter = 0;
        // Request image and store it within the base distance image.
        (*it)->camera_sensor->getDepthImage((*it)->image->data);
        base::samples::DistanceImage* image = (*it)->image;
        
        for(auto &p : (*it)->points) {
            size_t x_t = p.x;
            size_t y_t = p.y;
            Eigen::Matrix<double, 3, 1> scene_p;
            float pt = image->data[x_t+y_t*image->width];
            if(pt >= _minimum_distance.get() &&
                    pt <= _maximum_distance.get() &&
                    image->getScenePoint<double>( x_t, y_t, scene_p )) {
                // Rotate camera around the y axis.
                scene_p = (*it)->orientation * scene_p;
                // Transforms to robot frame (x: front, z: up) and adds to the pointcloud.
                scene_p = base::Vector3d(scene_p[2], -scene_p[0], -scene_p[1]);
                pointcloud.points.push_back(scene_p);
                counter++;
            }
        }
        RTT::log(RTT::Info) << counter << " points have been added by camera " << (*it)->name <<
                " ID "<< (*it)->sensor_id << RTT::endlog();
    }
    _pointcloud.write(pointcloud);
}
