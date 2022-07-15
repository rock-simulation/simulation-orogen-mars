require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

#Orocos.run 'mars::Task' => 'mars', 'mars::TMDS' => 'tmds' do
Orocos.run 'test_tmds' do

    mars = TaskContext.get 'mars'
    tmds = TaskContext.get 'tmds'
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

    mars.configure
    tmds.configure

    mars.start
    tmds.start
   
#    sleep 50
#
#    STDOUT.puts "Restartign mars"
#    mars.stop
#    mars.cleanup
#    mars.configure
#    mars.start

    Readline::readline("Press ENTER to quit") do
    end

end 
