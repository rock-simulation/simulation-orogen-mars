require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

Orocos.run 'crex_simulation', :output=>nil  do

    mars = TaskContext.get 'mars'
    xsens = TaskContext.get 'xsens'
    velodyne = TaskContext.get 'velodyne'
    ft_sensors = TaskContext.get 'ft_sensors'

    front_left_leg = TaskContext.get 'front_left_leg'
    front_right_leg = TaskContext.get 'front_right_leg'

    middle_left_leg = TaskContext.get 'middle_left_leg'
    middle_right_leg = TaskContext.get 'middle_right_leg'

    rear_left_leg = TaskContext.get 'rear_left_leg'
    rear_right_leg = TaskContext.get 'rear_right_leg'

    front_left_leg_contact = TaskContext.get 'front_left_leg_contact'
    front_right_leg_contact = TaskContext.get 'front_right_leg_contact'

    middle_left_leg_contact = TaskContext.get 'middle_left_leg_contact'
    middle_right_leg_contact = TaskContext.get 'middle_right_leg_contact'

    rear_left_leg_contact = TaskContext.get 'rear_left_leg_contact'
    rear_right_leg_contact = TaskContext.get 'rear_right_leg_contact'    

    mars.apply_conf_file("mars::Task.yml", ["default", "crex_in_dlr_scene"])
    mars.configure


    xsens.apply_conf_file("mars::IMU.yml", ["default"])
    xsens.configure

    velodyne.apply_conf_file("mars::RotatingLaserRangeFinder.yml", ["default"])
    velodyne.configure

    ft_sensors.apply_conf_file("mars::ForceTorque6DOF.yml", ["default"])
    ft_sensors.configure

    front_left_leg.apply_conf_file("mars::Joints.yml", ["default","front_left"])
    front_left_leg.configure
    front_right_leg.apply_conf_file("mars::Joints.yml", ["default","front_right"])
    front_right_leg.configure

    middle_left_leg.apply_conf_file("mars::Joints.yml", ["default","middle_left"])
    middle_left_leg.configure
    middle_right_leg.apply_conf_file("mars::Joints.yml", ["default","middle_right"])
    middle_right_leg.configure

    rear_left_leg.apply_conf_file("mars::Joints.yml", ["default","rear_left"])
    rear_left_leg.configure
    rear_right_leg.apply_conf_file("mars::Joints.yml", ["default","rear_right"])
    rear_right_leg.configure        

    front_left_leg_contact.apply_conf_file("mars::Joints.yml", ["default","front_left_contact"])
    front_left_leg_contact.configure
    front_right_leg_contact.apply_conf_file("mars::Joints.yml", ["default","front_right_contact"])
    front_right_leg_contact.configure

    middle_left_leg_contact.apply_conf_file("mars::Joints.yml", ["default","middle_left_contact"])
    middle_left_leg_contact.configure
    middle_right_leg_contact.apply_conf_file("mars::Joints.yml", ["default","middle_right_contact"])
    middle_right_leg_contact.configure

    rear_left_leg_contact.apply_conf_file("mars::Joints.yml", ["default","rear_left_contact"])
    rear_left_leg_contact.configure
    rear_right_leg_contact.apply_conf_file("mars::Joints.yml", ["default","rear_right_contact"])
    rear_right_leg_contact.configure      

    # Connections
    mars.start
    xsens.start
    velodyne.start
    ft_sensors.start

    front_left_leg.start
    front_right_leg.start

    middle_left_leg.start
    middle_right_leg.start

    rear_left_leg.start
    rear_right_leg.start

    front_left_leg_contact.start
    front_right_leg_contact.start
    
    middle_left_leg_contact.start
    middle_right_leg_contact.start

    rear_left_leg_contact.start
    rear_right_leg_contact.start
  
    Readline::readline("Press ENTER to quit") do
    end

end 
