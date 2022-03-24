#include "Task.hpp"
#include <mars/app/MARS.h>
#include <mars/sim/Simulator.h>
#include <mars/utils/Thread.h>
#include <mars/utils/mathUtils.h>
#include <mars/interfaces/sim/SimulatorInterface.h>

#include <mars/tasks/MarsControl.hpp>
#include <mars/gui/MarsGui.h>
#include <mars/main_gui/MainGUI.h>
#include <mars/main_gui/GuiInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
//#include <mars/graphics/GraphicsManager.h>
#include <mars/app/GraphicsTimer.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

#include <mars/sim/SimMotor.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>

//#include <mars/multisim-plugin/MultiSimPlugin.h>

#include <lib_manager/LibManager.hpp>
#include <QApplication>
#if QT_VERSION >= 0x050000
#include <QStyleFactory>
#else
#include <QPlastiqueStyle>
#endif

#include <envire_core/items/Transform.hpp>
#include <envire_core/items/Item.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <maps/grid/MLSMap.hpp>

#include <boost/filesystem.hpp>

#include <base-logging/Logging.hpp>

using namespace mars;
using mlsPrec = maps::grid::MLSMapPrecalculated;
using mlsKal = maps::grid::MLSMapKalman;

mars::interfaces::SimulatorInterface *Task::simulatorInterface = 0;
mars::Task *Task::taskInterface = 0;
mars::app::GraphicsTimer *Task::graphicsTimer = 0;
lib_manager::LibManager* Task::libManager = 0;

Task::Task(std::string const& name)
    : TaskBase(name)
    , multisimPlugin(0)
{
    Task::taskInterface = this;
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
    app = 0;
    _gravity.set(Eigen::Vector3d(0,0,-9.81));
    serialization_id = 0;
    last_serialization_id = 0;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
    , multisimPlugin(0)
{
    Task::taskInterface = this;
    app = 0;
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
    serialization_id = 0;
    last_serialization_id = 0;
}

Task::~Task()
{
    delete app;
}

void Task::startSimulation() {
    simulatorInterface->StartSimulation();
}

void Task::stopSimulation() {
    simulatorInterface->StopSimulation();
}


void Task::loadScene(::std::string const & path)
{
    if(simulatorInterface){
        simulatorInterface->loadScene(path, path,true,true);
    }else{
        LOG_ERROR_S << "Simulator not yet started cout not load scenefile";
    }
}

bool Task::loadSerializedPositions(::mars::SerializedScene const & serializedScene)
{

    //pause sim
    bool was_running=false;
    if(simulatorInterface->isSimRunning()){
        simulatorInterface->StopSimulation();
        was_running = true;
    }

    if (!serializedScene.has_objects){
        LOG_ERROR("EnvireMars adaptation missing here");
        // TODO: Code commented out to build, requires adaptation in the simulator core 
        // simulatorInterface->updateScenePositions(serializedScene.binary_scene);
        LOG_ERROR("EnvireMars adaptation missing here");
    }else{
        printf("loading objects unsupported\n");
    }

    //restart sim
    if(was_running){
        simulatorInterface->StartSimulation();
    }

    return true;
}

::mars::SerializedScene Task::serializePositions()
{
    ++serialization_id;

    //pause sim
    bool was_running=false;
    if(simulatorInterface->isSimRunning()){
        simulatorInterface->StopSimulation();
        was_running = true;
    }

    //serialize

    serialized_scene.id = serialization_id;
    serialized_scene.has_objects = false;
    LOG_ERROR("EnvireMars adaptation missing here");
    // TODO: fix the core of Envire Mars so that this functionality can be used again: serialized_scene.binary_scene = simulatorInterface->serializeScene(false);


    //restart sim
    if(was_running){
        simulatorInterface->StartSimulation();
    }

    return serialized_scene;
}


bool Task::loadState(boost::int32_t Id){
    if (savedStates[Id]){
        loadSerializedPositions(*(savedStates[Id]));
        return true;
    }
    return false;
}

boost::int32_t Task::saveState(){
    std::shared_ptr<mars::SerializedScene> scene = std::shared_ptr<mars::SerializedScene>(new mars::SerializedScene(serializePositions()));
    savedStates[scene->id] = scene;
    return scene->id;
}

bool Task::deleteState(boost::int32_t Id){
    if (savedStates.erase(Id)){
        return true;
    }
    return false;
}

mars::interfaces::SimulatorInterface* Task::getSimulatorInterface()
{
    return simulatorInterface;
}

mars::Task* Task::getTaskInterface(){
    return taskInterface;
}

void* Task::startTaskFunc(void* argument)
{
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
    TaskArguments* marsArguments = static_cast<TaskArguments*>(argument);

    Task* mars = marsArguments->mars;

    // Using the 'command-line' interface to pass
    // arguments to mars interface
    // set the option to "" if it does not require further args
    std::vector<Option> rawOptions = marsArguments->raw_options;

    if(marsArguments->controller_port > 0)
    {
        char buffer[10];
        sprintf(buffer, "%d", marsArguments->controller_port);
        Option controllerPortOption("-c", std::string(buffer));
        rawOptions.push_back(controllerPortOption);
    }

    if(!marsArguments->config_dir.empty())
    {
        Option confDirOption("-C", marsArguments->config_dir);
        rawOptions.push_back(confDirOption);
    }
    bool enable_gui = true;
    if(!marsArguments->enable_gui)
    {
      Option noGUIOption("--no-gui", "");
      rawOptions.push_back(noGUIOption);
      enable_gui = false;  
    }
    char** argv = mars->setOptions(rawOptions);
    int argc = mars->getOptionCount(rawOptions);
    // incrememt arcument counter since setOptions adds mars_core to arguments
    argc += 1;
    // Plus one for executable name
    for(int i = 0; i < argc; i++)
    {
        LOG_INFO_S << "Simulator: argument #" << i << " " << argv[i];
    }

    mars::app::MARS *simulation = new mars::app::MARS();
    simulation->readArguments(argc, argv);

    // Prepare Qt Application Thread which is required
    // for core mars and gui
    if(!Task::getTaskInterface()->app){
        //Initialize Qapplication only once! and keep the instance
        Task::getTaskInterface()->app = new QApplication(argc, argv, enable_gui);
        if (enable_gui)
        {        
#if QT_VERSION >= 0x050000
            QStyle* style = QStyleFactory::create("plastique");
            if(style)
            {
                Task::getTaskInterface()->app->setStyle(style);
            } else {
                LOG_WARN_S << "QStyle 'plastique' is not available";
            }
#else
            Task::getTaskInterface()->app->setStyle(new QPlastiqueStyle);
#endif
        }
    }

    setlocale(LC_ALL,"C");
    setenv("LANG","C",true);
    struct lconv* locale = localeconv();
    LOG_INFO_S << "Active locale (LC_ALL): ";

    if( *(locale->decimal_point) != '.')
    {
        LOG_ERROR_S << "Current locale conflicts with mars";
        marsArguments->failed_to_init = true;
        return 0;
    }

    std::string cmd;
    for(int i = 0; i < argc;++i)
    {
        cmd += std::string(argv[i]);
        cmd += " ";
    }
    LOG_INFO_S << "Starting mars with: " << cmd;
    simulation->start(argc, argv);

    // Prepare the LibManager and required configuration files
    libManager = simulation->getLibManager();

    // load the additionally specified plugins
    for( std::vector<std::string>::iterator it = marsArguments->mars_plugins.begin();
         it != marsArguments->mars_plugins.end(); ++it )
    {
        libManager->loadLibrary( *it );
    }

    mars->simulatorInterface = libManager->getLibraryAs<sim::Simulator>("mars_sim");
    if(!mars->simulatorInterface)
    {
        LOG_ERROR_S << "CRITICAL (cause abort) Simulation could not be retrieved via lib_manager";
        marsArguments->failed_to_init = true;
        return 0;
    }

    // set mars properties, if specified
    lib_manager::LibInterface* lib = libManager->getLibrary(std::string("cfg_manager"));
    if(lib)
    {
        cfg_manager::CFGManagerInterface* cfg = dynamic_cast<cfg_manager::CFGManagerInterface*>(lib);
        if(cfg){
            std::vector<SimulationProperty> props = marsArguments->mars_property_list;
            for(std::vector<SimulationProperty>::iterator prop_it = props.begin(); prop_it != props.end(); ++prop_it){
                // get or create property
                cfg_manager::cfgPropertyStruct cfg_prop_struct;
                cfg_prop_struct = cfg->getOrCreateProperty(prop_it->lib_name, prop_it->property_name, prop_it->value);

                // overriding any defaults
                cfg_prop_struct.sValue = prop_it->value;
                cfg->setProperty(cfg_prop_struct);
                LOG_DEBUG("setting property %s\n", cfg_prop_struct.sValue.c_str());
            }
        }
    }


    if(marsArguments->add_floor){
        mars->simulatorInterface->getControlCenter()->nodes->createPrimitiveNode("Boden",mars::interfaces::NODE_TYPE_PLANE,false,mars::utils::Vector(0,0,0.0),mars::utils::Vector(600,600,0));
    }
    int result = mars->simulatorInterface->getControlCenter()->dataBroker->registerTriggeredReceiver(mars,"mars_sim", "simTime","mars_sim/postPhysicsUpdate",1);
    (void)result;
    assert(result);

    // is realtime calc requested?
    if(marsArguments->realtime_calc){
        mars->simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "realtime calc", "value", marsArguments->realtime_calc);
    }

    if (enable_gui)
    {
        mars->marsGraphics = libManager->getLibraryAs<mars::interfaces::GraphicsManagerInterface>("mars_graphics");
    }

    // Synchronize with configureHook
    marsArguments->initialized = true;
    Task::getTaskInterface()->app->exec();


    libManager->releaseLibrary("mars_sim");
    libManager->releaseLibrary("cfg_manager");
    if (enable_gui)
    {
       libManager->releaseLibrary("mars_graphics");
    }

    delete simulation;
    //Do not delete the QApplication it does not like it to be restarted
    LOG_DEBUG_S << "Qapplication exec ended";

    return 0;
}

int Task::getOptionCount(const std::vector<Option>& options)
{
    std::vector<Option>::const_iterator it;

    // First just counting the number of arguments
    int count = 0;
    for(it = options.begin(); it != options.end(); it++)
    {
        Option option = *it;
        // Differentiate between option with args and without
        if(option.parameter != "")
            count += 2;
        else
            count += 1;
    }

    return count;
}

bool Task::setShow_coordinate_system(bool value)
{
    if(!marsGraphics){
        if(!_enable_gui.get())
        {
            return true;
        }
        else
        {
            LOG_ERROR("Could not change view of coordinate systems without an Graphics interface\n");
            return false;
        }
    }

    //Call the base function, DO-NOT Remove
    if(value)
        marsGraphics->hideCoords();
    else
        marsGraphics->showCoords();

    return(mars::TaskBase::setShow_coordinate_system(value));
}

bool Task::setReaction_to_physics_error(::std::string const & value)
{
    //TODO Add your code here
    if(isConfigured()){
        if(!simulatorInterface){
            LOG_ERROR("Task is not running could not set reaction to physics error");
            return false;
        }
        if(value == "abort" || value == "reset" || value == "warn" || value == "shutdown"){
            simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "onPhysicsError","value", value);
        }else{
            LOG_ERROR("Could not ser rection to physics: Possible Values: abort (killing sim), reset (ressing scene and mars), warn (keep mars running an print warnings), shutdown (stop physics but keep mars-running and set this tas to the error state)");
            return false;
        }
    }

    //Call the base function, DO-NOT Remove
    return(mars::TaskBase::setReaction_to_physics_error(value));
}

char** Task::setOptions(const std::vector<Option>& options)
{
    int count = getOptionCount(options)+ 1;
    char** argv = (char**) calloc(count, sizeof(char**));

    // Set executable name to mars_core
    count = 0;
    argv[count++] = "mars_core";

    std::vector<Option>::const_iterator it;
    for(it = options.begin(); it != options.end(); it++)
    {
        Option opt = *it;

        if(opt.name == "")
            continue;

        argv[count++] = strdup(opt.name.c_str());
        if(opt.parameter != "")
        {
            argv[count++] = strdup(opt.parameter.c_str());
        }
    }

    return argv;
}

bool Task::configureHook()
{
    if(_config_dir.get().empty())
    {
        LOG_ERROR_S << "Config directory is not set! Cannot start mars.";
        throw std::runtime_error("Config directory is not set! Can not start mars");
    }


    //check if the environemnt was sourced more than once and the path has more than one entry
    int pos = _config_dir.get().rfind(":/");
    if(pos != _config_dir.get().size()-1)
        _config_dir.set(_config_dir.get().substr(pos+1));

    LOG_INFO_S << "Calling configure: with " << _config_dir.get();

    //mars is not setting the config path properly
    //therefore we have to go into to the config folder
    //if(0 != chdir(_config_dir.get().c_str()))
    //{
    //    LOG_ERROR_S << "Config directory " << _config_dir.get() << " does not exist. Cannot start mars.";
    //    throw std::runtime_error(std::string("Config directory ") +_config_dir.get() +" does not exist. Can not start mars.");
    //}

    // Startup of mars
    TaskArguments argument;
    argument.mars = this;
    argument.enable_gui = _enable_gui.get();
    argument.controller_port = _controller_port.get();
    argument.raw_options = _raw_options.get();
    argument.config_dir = _config_dir.get();
    argument.mars_property_list = _simulation_property_list.get();
    argument.initialized = false;
    argument.add_floor = _add_floor.get();
    argument.failed_to_init=false;
    argument.realtime_calc = _realtime_calc.get();
    argument.mars_plugins = _plugins.get();

    // go through list of plugins, and see if they have an absolute path
    for( std::vector<std::string>::iterator it = argument.mars_plugins.begin();
          it != argument.mars_plugins.end(); ++it )
    {
        *it = boost::filesystem::absolute(
              boost::filesystem::path( *it ),
              boost::filesystem::path( _plugin_dir.get() ) ).string();
    }

    int ret = pthread_create(&thread_info, NULL, startTaskFunc, &argument);
    if(ret)
    {
        LOG_ERROR_S << "Failed to create MARS thread: pthread error " << ret;
        throw std::runtime_error("Failed to create MARS thread");
    }

    for(int i=0; !argument.initialized && !argument.failed_to_init;++i)
    {
        //give up after 10 sec
        if(i > 1000)
        {
            LOG_ERROR_S << "Cannot start mars thread";
            throw std::runtime_error("Cannot start mars thread!");
        }
        usleep(10000);
    }
    if(argument.failed_to_init){
            LOG_ERROR_S << "Task failed to start, see Error above";
            return false;
    }

    LOG_INFO_S << "Task running";

    // Simulation is now up and running and plugins can be added
    // Configure basic functionality of mars
    // Check if distributed mars should be activated

    // todo: should be loaded via lib_manager
    /*
    if(_distributed_mars.get())
    {
        LOG_INFO_S << "Loading MultiSimPlugin";
        multisimPlugin = new MultiSimPlugin(libManager);
        LOG_INFO_S << "MultiSimPlugin loaded";
    }
    */
    // Load scenes before robot to avoid complex robots blocking correct loading of the scene
    std::vector<std::string> sceneNames = _initial_scenes.get();
    if(!sceneNames.empty()){
        for (std::vector< std::string >::iterator scene = sceneNames.begin(); scene != sceneNames.end();scene++){
            simulatorInterface->loadScene(*scene, *scene,true,true);
        }
    }

    if(!_initial_scene.get().empty()){
        simulatorInterface->loadScene(_initial_scene.get(), std::string("initial"),true,true);
    }

    std::vector<Positions> positions = _positions.get();
    if(!positions.empty()){
        for (std::vector< Positions >::iterator offset = positions.begin(); offset != positions.end();offset++){
            move_node(*offset);
        }
    }    

    std::vector<mars::SceneConfig> scene_configs = _scene_setup.get();
    if(!scene_configs.empty()){
        std::cout << "NOT EMPTY" << std::endl;
        for (std::vector<mars::SceneConfig>::iterator scene = scene_configs.begin(); scene != scene_configs.end(); scene++){
            utils::Vector pos;
            pos.x() = scene->posx;
            pos.y() = scene->posy;
            pos.z() = scene->posz;
            
            utils::Vector rot;
            rot.x() = scene->rotx;
            rot.y() = scene->roty;
            rot.z() = scene->rotz;

            std::cout << "LOAD: " << scene->path << " " << scene->name << std::endl;

            simulatorInterface->loadScene(scene->path, scene->name, pos, rot, true, true);
        }
    }    





    mars::Pose initial_pose = _initial_pose.get();
    if(!initial_pose.empty()){
        mars::interfaces::ControlCenter* control = simulatorInterface->getControlCenter();
        if (control){
            for (mars::Pose::iterator pos = initial_pose.begin(); pos != initial_pose.end();pos++){
                int marsMotorId = control->motors->getID( pos->name );
                mars::sim::SimMotor *motor = control->motors->getSimMotor( marsMotorId );
                if (motor){
                    motor->setValue( pos->pos );
                }else{
                    LOG_ERROR("no motor %s",pos->name.c_str());
                }
            }
        }else{
            LOG_ERROR("no contol center");
        }
    }


    {//Setting the Step-with for the mars
    cfg_manager::cfgPropertyStruct c = simulatorInterface->getControlCenter()->cfg->getOrCreateProperty("Simulator", "calc_ms", _sim_step_size.get()*1000.0);
    c.dValue = _sim_step_size.get()*1000.0;
    simulatorInterface->getControlCenter()->cfg->setProperty(c);
    }


    {
    std::string value = _reaction_to_physics_error.get();
    if(value == "abort" || value == "reset" || value == "warn" || value == "shutdown"){
        simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "onPhysicsError","value", value);
    }else{
        LOG_ERROR("Wront selection for physic error setting\n");
        return false;
    }
    }

    simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator","getTime:useNow","value",_use_now_instead_of_sim_time.get());

    setGravity_internal(_gravity.get());

    return updateDynamicProperties();
}


bool Task::startHook()
{
    // Simulation should be either started manually,
    // or by using the control_action input_port
    //
    if (_start_sim.get()){
        simulatorInterface->StartSimulation();
    }
    return true;
}

void Task::updateHook()
{
    mars::Control controlAction;
    if(_control_action.read(controlAction) == RTT::NewData)
    {
        switch(controlAction)
        {
            case START:
                LOG_INFO_S << "ControlAction: Start received";
                if(!simulatorInterface->isSimRunning())
                    simulatorInterface->startStopTrigger();
                break;
            case PAUSE:
                LOG_INFO_S << "ControlAction: Pause received";
                if(simulatorInterface->isSimRunning())
                    simulatorInterface->startStopTrigger();
                break;
            case RESET:
                LOG_INFO_S << "ControlAction: Reset received";
                simulatorInterface->resetSim();
                break;
            case STEP:
                LOG_INFO_S << "ControlAction: Step received";
                simulatorInterface->singleStep();
                break;
            default:
                LOG_WARN_S << "Simulation: Unknown control action " << controlAction << " received";

        }
    }

    if(simulatorInterface->hasSimFault()){
        LOG_INFO_S << "Simulation detected a Physics error, stopping all plugins and go to Exception state";
        for(unsigned int i=0;i<plugins.size();i++){
            plugins[i]->handleMarsShudown();
        }
        exception(PHYSICS_ERROR);
 //       QCoreApplication::quit(); //Quitting QApplication too
    }

    if (serialization_id != last_serialization_id){
        _serialized_scene.write(serialized_scene);
        last_serialization_id = serialization_id;
    }

    if (_updatePositions.read(serialized_scene_in) == RTT::NewData){
        printf("%s update pos\n",__PRETTY_FUNCTION__);
        loadSerializedPositions(serialized_scene_in);
    }
}

void Task::errorHook()
{
}

void Task::stopHook()
{
    /*
    std::cout << "STOP HOOK" << std::endl;
    for(unsigned int i=0;i<plugins.size();i++){
        plugins[i]->handleTaskShudown();
    }
    simulatorInterface->exitTask();

    std::cout << "STOP HOOK quitting qapp" << std::endl;
    QCoreApplication::quit(); //Quitting QApplication too
    std::cout << "STOP HOOK quitting qapp finish" << std::endl;
    */
}

void Task::registerPlugin(Plugin* plugin){
    plugins.push_back(plugin);
}

void Task::unregisterPlugin(Plugin* plugin){
    plugins.push_back(plugin);
}

void Task::cleanupHook()
{
    for(unsigned int i=0;i<plugins.size();i++){
        plugins[i]->handleMarsShudown();
    }
    plugins.clear();

    simulatorInterface->exitMars();
    while( simulatorInterface->isSimRunning()) ;

    if (app){
    	app->quit();
    }



    LOG_DEBUG_S << "CLEANUP HOOK quitting qapp finish";

   // delete libManager;

//    libManager->releaseLibrary("mars_sim");
//    libManager->releaseLibrary("mars_gui");
//    libManager->releaseLibrary("mars_graphics");
//    libManager->releaseLibrary("gui_core");


 //   if(multisimPlugin) delete multisimPlugin;
}
/*
bool Task::recover(){
    std::cout << "RECOVER HOOK" << std::endl;
    return TaskBase::recover();
}
void Task::fatal(){
    std::cout << "FATAL HOOK" << std::endl;
    TaskBase::fatal();
}
void Task::exception(){
    std::cout << "EXCEPTION HOOK" << std::endl;
    TaskBase::exception();
}
*/

void Task::receiveData(
        const data_broker::DataInfo& info,
        const data_broker::DataPackage& package,
        int id)
{
    _simulated_time.write(base::Time::fromMilliseconds(simulatorInterface->getTime()));

    if (_frame_name.get() != "") {
        mars::interfaces::ControlCenter* control = simulatorInterface->getControlCenter();
        if (control && simulatorInterface->isSimRunning()){        
            LOG_ERROR("EnvireMars adaptation missing here");
            /* TODO envireMars fix: code commented out for build purposes
            if (control->graph != NULL) {
                if (control->graph->containsFrame(_frame_name.get()) == true) {
                    std::cout << "GET Transform" << std::endl;
                    envire::core::Transform frame_trasf = control->graph->getTransform(SIM_CENTER_FRAME_NAME, _frame_name.get());

                    base::samples::RigidBodyState frame_rbg;
                    frame_rbg.time = base::Time::now();
                    frame_rbg.sourceFrame = SIM_CENTER_FRAME_NAME;
                    frame_rbg.targetFrame = _frame_name.get();
                    frame_rbg.position = frame_trasf.transform.translation;
                    frame_rbg.orientation = frame_trasf.transform.orientation;

                    _frame_pose.write(frame_rbg);
                }
            }
            */

        }
    }    
}

bool Task::setGravity_internal(::base::Vector3d const & value){
    simulatorInterface->setGravity(value);
    return true;
}

bool Task::setGravity(::base::Vector3d const & value)
{
 if(!isConfigured()){
     //The configuration will be done within the configure hook later
     return(mars::TaskBase::setGravity(value));
 }

 setGravity_internal(value);
 return(mars::TaskBase::setGravity(value));
}

void Task::setPosition(::mars::Positions const & positions)
{
    if(isRunning() || isConfigured()){
        LOG_DEBUG("moving '%s' to %g/%g/%g\n", positions.nodename.c_str(), positions.posx, positions.posy, positions.posz);
        move_node(positions);
    }else{
        LOG_ERROR("setPosition called, but mars::Task is whether configured nor running ");
    }
    return;
}

void Task::getMLSMap() {
    std::cout << "Task::getMLSMap()" << std::endl;
    mars::interfaces::ControlCenter* control = simulatorInterface->getControlCenter();
    if (control){    
        LOG_ERROR("EnvireMars adaptation missing here");
        /*
        envire::core::GraphTraits::vertex_descriptor mls_vertex = control->graph->getVertex(MLS_FRAME_NAME);
        if (control->graph->containsItems<envire::core::Item<mlsPrec>>(mls_vertex)){
            std::cout << "Task::getMLSMap() CONTAINT" << std::endl;

            using IteratorMLS = envire::core::EnvireGraph::ItemIterator<envire::core::Item<mlsPrec>>;
            IteratorMLS begin_sim, end_sim;
            boost::tie(begin_sim, end_sim) = control->graph->getItems<envire::core::Item<mlsPrec>>(mls_vertex);
            for (;begin_sim!=end_sim; begin_sim++)
            {
                std::cout << "Task::getMLSMap() MLS" << std::endl;

                maps::grid::MLSMapKalman mlsKalman;
                mlsKalman.time = base::Time::now();
                mlsKalman.frame_id = MLS_FRAME_NAME;

                //mlsPrec mlsP = begin_sim->getData();
                //mlsKal mlsKAux = mlsP;
                //mlsKalman.data = mlsKAux;

                mlsKalman.data = mls_dummy_fix;

                // TODO: this is quick fix
                _mls_map.write(mlsKalman);
            }        
        }
        */
    }
}

bool Task::prepareGraphForMLS()
{
    bool ok;
    ok = true;
    mars::interfaces::ControlCenter* control = simulatorInterface->getControlCenter();
    if (control){
        LOG_DEBUG("[Task::prepareGraphForMLS] Mars control center available");
        // TODO: take it out into the load mls plugin
        // Load the mls in the graph
        LOG_ERROR("EnvireMars adaptation missing here");
        /* TODO: Add the code here, so that the functionality is recovered
        envire::core::FrameId mlsFrameId = MLS_FRAME_NAME; 
        envire::core::FrameId centerFrameId = SIM_CENTER_FRAME_NAME;
        envire::core::Transform mlsTf;
        mlsTf.setIdentity();
        //mlsTf.transform.translation << 3.0, 5.0, 7.0; Different positions of the mls don't affect where it is visualized
        if (not(control->graph->containsFrame(mlsFrameId))){
            control->graph->addFrame(mlsFrameId);
            control->graph->addTransform(mlsFrameId, centerFrameId, mlsTf);
            LOG_DEBUG("[Task::prepareGraphForMLS] Added MLS frame transformation: %g, %g, %g", mlsTf.transform.translation.x(), mlsTf.transform.translation.y(), mlsTf.transform.translation.z());
        }
        if (not(control->graph->containsEdge(mlsFrameId, centerFrameId))){
            control->graph->addTransform(mlsFrameId, centerFrameId, mlsTf);
            LOG_DEBUG("[Task::prepareGraphForMLS] The frame exists, but not the tf");
        }
        else{
            control->graph->updateTransform(mlsFrameId, centerFrameId, mlsTf);
            LOG_DEBUG("[Task::prepareGraphForMLS] Updated MLS frame transformation: %g, %g, %g", mlsTf.transform.translation.x(), mlsTf.transform.translation.y(), mlsTf.transform.translation.z());
        }
        */
    }
    else{
        LOG_ERROR("[Task::prepareGraphForMLS] No contol center");
        ok = false;
    }
    return ok;
}

void Task::loadRobot(const base::samples::RigidBodyState& robotPose)
{
    mars::interfaces::ControlCenter* control = simulatorInterface->getControlCenter();
    if (control){
        // Take the robot root link frame and move it to the target Pose
        envire::core::Transform robotTf(robotPose.position, robotPose.orientation);
        LOG_DEBUG("[Task::loadRobot] Robot Target Pose: %g, %g, %g", robotTf.transform.translation.x(), robotTf.transform.translation.y(), robotTf.transform.translation.z());

        LOG_ERROR("EnvireMars adaptation missing here");
        /* TODO: add code here to recover functionality
        envire::core::FrameId robotRootFrame = ROBOT_ROOT_LINK_NAME;
        control->nodes->setTfToCenter(robotRootFrame, robotTf);
        */
        LOG_DEBUG("[Task::loadRobot] Robot moved");
    }
    else{
        LOG_ERROR("[Task::loadRobot] No contol center");
    }
}

void Task::setupMLSSimulation(const base::samples::RigidBodyState& robotPose, const envire::core::SpatioTemporal<maps::grid::MLSMapKalman > & mls)
{
    mars::interfaces::ControlCenter* control = simulatorInterface->getControlCenter();
    if (control){
        LOG_DEBUG("[Task::setupMLSSimulation] Method called!");
        if(prepareGraphForMLS())
        {

            LOG_ERROR("EnvireMars adaptation missing here");
            /* TODO: add this code so that functionality is recovered
            maps::grid::MLSMapKalman mlsKalST = mls;
            // TODO: this is quick fix
            mls_dummy_fix = mlsKalST.getData();
            mlsKal mlsKAux = mlsKalST.getData();
            mlsPrec mlsP = mlsKAux;
            */
            //envire::core::Item<mlsPrec>::Ptr mlsItemPtr(new envire::core::Item<mlsPrec>(mlsP));

            //control->graph->addItemToFrame(mlsFrameId, mlsItemPtr);
            LOG_DEBUG("[Task::setupMLSSimulation] MLS added");
            //loadRobot(robotPose);
        }
        else
        {
            LOG_ERROR("[Task::setupMLSSimulation] MLS Could not be loaded");
        }
    }
    else{
        LOG_ERROR("[Task::setupMLSSimulation] No contol center");
    }
    return;
}

void Task::setupMLSPrecSimulation(const base::samples::RigidBodyState& robotPose, const envire::core::SpatioTemporal<maps::grid::MLSMapPrecalculated > & mls)
{
    mars::interfaces::ControlCenter* control = simulatorInterface->getControlCenter();
    if (control){
        if (prepareGraphForMLS()){
            //envire::core::SpatioTemporal<maps::grid::MLSMapPrecalculated > spatioTemporal;
            LOG_ERROR("EnvireMars adaptation missing here");
            /*
            envire::core::Item<mlsPrec>::Ptr mlsItemPtr(new envire::core::Item<mlsPrec>(mls.data));
            //envire::core::Item<mlsPrec>::Ptr mlsItemPtr(&mls);
            control->graph->addItemToFrame(mlsFrameId, mlsItemPtr);
            LOG_DEBUG("[Task::setupMLSPrecSimulation] MLS added");
            */
            loadRobot(robotPose);
        }
        else
        {
            LOG_ERROR("[Task::setupMLSPrecSimulation] MLS Could not be loaded");
        }
    }
    else{
        LOG_ERROR("[Task::setupMLSSimulation] No contol center");
    }
    return;
}

bool Task::setSim_step_size(double value)
{
    //convert to ms
    value *= 1000.0;
    if(!isConfigured()){
        //The configuration will be done within the configure hook later
        return(mars::TaskBase::setSim_step_size(value));
    }
    cfg_manager::cfgPropertyStruct c = simulatorInterface->getControlCenter()->cfg->getOrCreateProperty("Simulator", "calc_ms", value);
    c.dValue = value;
    simulatorInterface->getControlCenter()->cfg->setProperty(c);
    return(mars::TaskBase::setSim_step_size(value));
}

void Task::move_node(::mars::Positions const & arg)
{
    mars::interfaces::NodeManagerInterface* nodes = simulatorInterface->getControlCenter()->nodes;
    mars::interfaces::NodeId id = nodes->getID(arg.nodename);
    if (id){
        mars::interfaces::NodeData nodedata = nodes->getFullNode(id);
        utils::Vector pos = nodes->getPosition(id);

        pos.x() = arg.posx;
        pos.y() = arg.posy;
        pos.z() = arg.posz;

        mars::utils::Vector rotoff;

        rotoff.x() =  arg.rotx;
        rotoff.y() =  arg.roty;
        rotoff.z() =  arg.rotz;

        mars::utils::Quaternion newrot = mars::utils::eulerToQuaternion(rotoff);

        nodedata.pos = pos;
        nodedata.rot = newrot;

        if (arg.edit_all){
            nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_POS | mars::interfaces::EDIT_NODE_MOVE_ALL);
            nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_ROT | mars::interfaces::EDIT_NODE_MOVE_ALL);
        }else{
            printf("edit node only %s\n",arg.nodename.c_str());
            nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_POS);
            nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_ROT);
        }
    }else{
        LOG_ERROR("[MarsTask::move_node]node '%s' unknown\n", arg.nodename.c_str());
    }
}
