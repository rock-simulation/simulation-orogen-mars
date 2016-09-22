require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

Orocos.run 'crex_simulation', :output=>nil  do

    mars = TaskContext.get 'mars'
    xsens = TaskContext.get 'xsens'
    velodyne = TaskContext.get 'velodyne'
    ft_sensors = TaskContext.get 'ft_sensors'

    mars.apply_conf_file("mars::Task.yml", ["default", "crex_in_dlr_scene"])
    mars.configure
    mars.start

    xsens.apply_conf_file("mars::IMU.yml", ["default"])
    xsens.configure

    velodyne.apply_conf_file("mars::RotatingLaserRangeFinder.yml", ["default"])
    velodyne.configure

    ft_sensors.apply_conf_file("mars::ForceTorque6DOF.yml", ["default"])
    ft_sensors.configure

    # Connections
    xsens.start
    velodyne.start
    ft_sensors.start
  
    Readline::readline("Press ENTER to quit") do
    end

end 
