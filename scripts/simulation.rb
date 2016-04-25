require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

Orocos.run 'simulation_asguard' do
    
    #TaskContext 'velodyne' do
    #    subclasses  'mars::Task'
    #end

    mars = TaskContext.get 'mars'
    velodyne = TaskContext.get 'velodyne'
    actuators = TaskContext.get 'mars_actuators'
    dispatcher = TaskContext.get 'joint_dispatcher'
    #sysmon = TaskContext.get 'sysmon'
    

    #velodyne.subclasses('mars')

#    mars.controller_port = 1600
#    mars.enable_gui = 1

#    option_t = Orocos.registry.get 'simulation/Option'
#    option = option_t.new
#    option.name = "-c"
#    option.parameter = "1601"
#
#    raw_options = mars.raw_options
#    raw_options << option
#
#    mars.raw_options = raw_options
    
    mars.apply_conf_file("mars::Task.yml", ["default"])
    mars.configure
    mars.start

    
    sleep 10
    
    # I am not sure about the configs, e.g. maybe the default have to be applied too
    
    dispatcher.apply_conf_file("joint_dispatcher::Task.yml", ["default"])
    dispatcher.configure
    
    #sysmon.apply_conf_file("mars::Joints.yml", ["default"])
    #sysmon.apply_conf_file("mars::Joints.yml", ["sysmon"])
    #This one is for the passive joint, so it assumes there is a motor there, which we don't have anymore in the model. Do we still neeed this task?
    #sysmon.configure
    
    actuators.apply_conf_file("mars::Joints.yml", ["base"]) 
    actuators.configure
    
    velodyne.apply_conf_file("mars::RotatingLaserRangeFinder.yml", ["default"])
    velodyne.configure
    
    # Connections
    dispatcher.command_out.connect_to(actuators.command)
    actuators.status_samples.connect_to(dispatcher.status_samples_in, :type=>:buffer, :size=>100)
    #sysmon.status_samples.connect_to(dispatcher.body_joint_in, :type=>:buffer, :size=>100)
    

    dispatcher.start
    #sysmon.start
    actuators.start
    velodyne.start
    

    
    #
#    STDOUT.puts "Restartign mars"
#    mars.stop
#    mars.cleanup
#    mars.configure
#    mars.start

    Readline::readline("Press ENTER to quit") do
    end

end 
