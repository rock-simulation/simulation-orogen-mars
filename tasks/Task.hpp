#ifndef SIMULATION_MARS_TASK_HPP
#define SIMULATION_MARS_TASK_HPP

#include "mars/TaskBase.hpp"
#include <vector>
#include <mars/data_broker/ReceiverInterface.h>
#include "Plugin.hpp"
#include <boost/thread/mutex.hpp>
#include "mars/sceneType.hpp"
#include <maps/grid/MLSMap.hpp> 
#include <base/Pose.hpp>

class QApplication;

/** From MARS */
//
namespace lib_manager{
  class LibManager;
};

namespace mars{
    namespace interfaces{
        class SimulatorInterface;
        class PluginInterface;
        class GraphicsManagerInterface;
    };
    namespace app{
        class GraphicsTimer;
    };
};


namespace mars {

    class Task;

    // Argument to pass to startTaskFunc 
    struct TaskArguments
    {
        Task* mars;
        bool enable_gui;
        int controller_port;
        std::vector<SimulationProperty> mars_property_list;
        std::vector<std::string> mars_plugins;
        std::string config_dir;
        bool initialized;
        bool add_floor;
        bool failed_to_init;
        bool realtime_calc;
        // Raw command line option can be passed to mars
        // using this option vector
        std::vector<Option> raw_options;
    };


    /**
    * Core module that brings up the mars mars and
    * makes it accessible as a orogen module
    *
    * use subclassing to derive robot specific modules, e.g.
    * 
    * task_context 'RobotSimulation' do
    *         subclasses 'mars::Task'
    * ..
    * end
    *
    */
    class Task : public TaskBase, public mars::data_broker::ReceiverInterface

    {
	friend class TaskBase;
    protected:
        QApplication* app; 
    	static mars::app::GraphicsTimer *graphicsTimer;
	static mars::interfaces::SimulatorInterface* simulatorInterface;
	static mars::Task* taskInterface;
	static void* startTaskFunc(void *);
        static std::string configDir;
	static bool marsRunning;
	int serialization_id, last_serialization_id;
	mars::SerializedScene serialized_scene;
	mars::SerializedScene serialized_scene_in;

	std::map<int , std::shared_ptr<mars::SerializedScene> > savedStates;


	pthread_t thread_info; 
	static lib_manager::LibManager* libManager;

        mars::interfaces::PluginInterface* multisimPlugin;

        int getOptionCount(const std::vector<Option>& options);
        
        /* This operation moves a node to a specific position, simpliar to the positions property but can be used during runtime
         */
        virtual void move_node(::mars::Positions const & arg);

        char** setOptions(const std::vector<Option>& options);

        /* Handler for the loadScene operation
         */
        virtual void loadScene(::std::string const & path);

        /* expects previously loaded objects, just updates the positions
         */
        virtual bool loadSerializedPositions(::mars::SerializedScene const & serializedScene);

        /* load simulation state by identifier and return whether it was found
         */
        virtual bool loadState(boost::int32_t Id);

        /* store current simulation state and returns an identifier
         */
        virtual boost::int32_t saveState();

        /* delete simulation state by identifier and return whether it was found
         */
        virtual bool deleteState(boost::int32_t Id);

        virtual void startSimulation();
        virtual void stopSimulation();

        /* serializes the frames and writes them to the serialized_scene output port
         */
        virtual ::mars::SerializedScene serializePositions();

        std::vector<Plugin*> plugins;
        
        /* Dynamic Property setter of show_coordinate_system
         */
        virtual bool setShow_coordinate_system(bool value);
        
        /* Dynamic Property setter of reaction_to_physics_error
         */
        virtual bool setReaction_to_physics_error(::std::string const & value);

        

        // GraphicsTimer will be later called with the marsGraphics reference
        // which can be also NULL for a disabled gui
        mars::interfaces::GraphicsManagerInterface* marsGraphics;
        
        virtual bool setSim_step_size(double value);
        virtual bool setGravity(::base::Vector3d const & value);
        virtual bool setGravity_internal(::base::Vector3d const & value);
        virtual void setPosition(::mars::Positions const & positions);
        virtual void setupMLSSimulation(const base::samples::RigidBodyState& robotPose, const envire::core::SpatioTemporal<maps::grid::MLSMapKalman > & mls);


        //virtual void setupMLSSimulation(base::Pose const &robotPose, maps::grid::MLSMapKalman const & mls); 

    public:
	/** get the singleton instance of the simulator interface
	 */
	static mars::interfaces::SimulatorInterface* getSimulatorInterface();
	static mars::Task* getTaskInterface();

        Task(std::string const& name = "mars::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);

	~Task();

        void registerPlugin(Plugin* plugin);
        void unregisterPlugin(Plugin* plugin);

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
         bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
         bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called. See README.txt for different
         * triggering options.
         *
         * The warning(), error() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeWarning, RunTimeError and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recovered()
         * allows you to go back into the Running state.  In the second case,
         * the errorHook() will be called instead of updateHook() and in the
         * third case the component is stopped and resetError() needs to be
         * called before starting it again.
         *
         */
         void updateHook();
        

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recovered() to go back in the Runtime state.
         */
         void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
         void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
         void cleanupHook();
    
         void receiveData(const mars::data_broker::DataInfo& info,const mars::data_broker::DataPackage& package,int id);
        
    };
}

#endif

