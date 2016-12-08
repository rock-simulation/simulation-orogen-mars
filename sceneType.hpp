#ifndef MARS_SCENE_TYPE_HH
#define MARS_SCENE_TYPE_HH

#include <vector>
#include <string>

namespace mars 
{

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
