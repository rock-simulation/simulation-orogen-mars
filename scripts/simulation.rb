require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

Orocos.run 'simulation_asguard' do

    mars = TaskContext.get 'mars'
    mars_actuators = TaskContext.get 'mars_actuators'
    sysmon = TaskContext.get 'sysmon'
    joint_dispatcher = TaskContext.get 'joint_dispatcher'
    simple_controller = TaskContext.get 'simple_controller'
    xsens = TaskContext.get 'xsens'
    odometry = TaskContext.get 'odometry'
    velodyne = TaskContext.get 'velodyne'
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
    # I am not sure about the configs, e.g. maybe the default have to be applied too
    mars.apply_conf_file("mars::Task.yml", ["default"])
    mars.configure
    mars.start
    #sleep 10
    mars_actuators.apply_conf_file("mars::Joints.yml", ["base"]) 
    mars_actuators.configure
    sysmon.apply_conf_file("mars::Joints.yml", ["sysmon"])
    sysmon.configure
    joint_dispatcher.apply_conf_file("joint_dispatcher::Task.yml", ["default"])
    joint_dispatcher.configure
    simple_controller.apply_conf_file("skid4_control::SimpleController.yml", ["default"])
    simple_controller.configure
    xsens.apply_conf_file("mars::IMU.yml", ["default"])
    xsens.configure
    odometry.apply_conf_file("odometry::Skid.yml", ["asguard"])
    odometry.configure
    velodyne.apply_conf_file("mars::RotatingLaserRangeFinder.yml", ["default"])
    velodyne.configure
    # Connections
    joint_dispatcher.command_out.connect_to(mars_actuators.command, :type=>:buffer, :size=>100)
    mars_actuators.status_samples.connect_to(joint_dispatcher.status_samples_in, :type=>:buffer, :size=>100)
    sysmon.status_samples.connect_to(joint_dispatcher.body_joint_in, :type=>:buffer, :size=>100)
    simple_controller.command.connect_to(joint_dispatcher.command, :type=>:buffer, :size=>100)
    joint_dispatcher.status_samples.connect_to(simple_controller.status_samples, :type=>:buffer, :size=>100)
    joint_dispatcher.status_samples.connect_to(odometry.actuator_samples, :type=>:buffer, :size=>100)
    xsens.orientation_samples.connect_to(odometry.dynamic_transformations, :type=>:buffer, :size=>100)
    mars_actuators.start
    sysmon.start
    joint_dispatcher.start
    simple_controller.start
    xsens.start
    odometry.start
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
