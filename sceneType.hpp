#ifndef MARS_SCENE_TYPE_HH
#define MARS_SCENE_TYPE_HH

#include <vector>
#include <string>
#include <stdint.h>

namespace mars 
{

struct SerializedScene{
    bool has_objects;
    unsigned int id;
    std::vector <uint8_t> binary_scene;

};

struct SceneConfig {
		std::string name;
		std::string path;
		double posx;
		double posy;
		double posz;
		double rotx;
		double roty;
		double rotz;
	};
}
#endif
