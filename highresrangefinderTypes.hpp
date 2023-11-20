
#ifndef MARS_HIGHRESRANGEFINDER_TYPES_HH
#define MARS_HIGHRESRANGEFINDER_TYPES_HH

#include <string>

namespace mars
{

struct HighResRangeFinderCamera {
	std::string name;   ///<Name of the camera within the scene file
	double orientation; ///<To get a full 360 degree view you have to add four 90-cameras with 0, 90, 180 and 270
};

}
#endif


