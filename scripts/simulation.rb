require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

Orocos.run 'simulation_asguard' do

    mars = TaskContext.get 'mars'
    velodyne = TaskContext.get 'velodyne'
    mars_actuators = TaskContext.get 'mars_actuators'
    joint_dispatcher = TaskContext.get 'joint_dispatcher'
    xsens = TaskContext.get 'xsens'
    controller = TaskContext.get 'simple_controller'
    body_state = TaskContext.get 'body_state'
    
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
    
    joint_dispatcher.apply_conf_file("joint_dispatcher::Task.yml", ["default"])
    joint_dispatcher.configure
    
    mars_actuators.apply_conf_file("mars::Joints.yml", ["base"]) 
    mars_actuators.configure
    
    velodyne.apply_conf_file("mars::RotatingLaserRangeFinder.yml", ["default"])
    velodyne.configure
    
    xsens.apply_conf_file("mars::IMU.yml", ["default"])
    xsens.configure
    
    body_state.apply_conf_file("asguard::BodyTask.yml", ["default"])
    body_state.configure

    controller.apply_conf_file("skid4_control::SimpleController.yml", ["default"])
    controller.configure
    
    # Connections
    joint_dispatcher.command_out.connect_to(mars_actuators.command, :type=>:buffer, :size=>100)
    mars_actuators.status_samples.connect_to(joint_dispatcher.status_samples_in, :type=>:buffer, :size=>100)
    
    joint_dispatcher.status_samples.connect_to(body_state.actuator_samples, :type => :buffer, :size => 100)
    #body_state.contact_samples.connect_to(odometry.contact_samples, :type=>:buffer, :size=>100)

    controller.command.connect_to(joint_dispatcher.command, :type=>:buffer, :size=>100)
    joint_dispatcher.status_samples.connect_to(controller.status_samples, :type=>:buffer, :size=>100)

    body_state.start 
    xsens.start
    joint_dispatcher.start
    mars_actuators.start
    velodyne.start
    controller.start
    
    #
#    STDOUT.puts "Restartign mars"
#    mars.stop
#    mars.cleanup
#    mars.configure
#    mars.start

    Readline::readline("Press ENTER to quit") do
    end

end 
