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
    
    velodyne.apply_conf_file("mars::RotatingLaserRangeFinder.yml", ["default"])
    velodyne.configure
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
