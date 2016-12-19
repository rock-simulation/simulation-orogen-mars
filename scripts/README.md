## asguard_tests.rb
### Behavior description
Asguard moves forward and produces Lidar sensor data.
### Deployments
Runs the deployment  'asguard_tests' (mars.orogen), the simple controller and the joint dispatcher.

## asguard_velodyne_no_gui.rb
### Behavior description
Asguard is staying at initial position and produces Lidar sensor data. No gui should be started.
### Deployments
Runs the deployment  'asguard_tests' (mars.orogen), the simple controller and the joint dispatcher.

## config
Config files for the different scripts.

## crex_simulation.rb
### Behavior description
Crex legs are connected, velodyne started and xsens also runnning. Feet sensor values are also available.
### Deployments
Runs the deployment 'crex_simulation' (entern.orogen).

## crex_tests.rb
### Behavior description
Crex moves forward and produces Lidar sensor data.
#TODO Crex does not move; "crexLegInterpolator", "motion_control", and "joint_dispatcher" are not connnected, configured or started
### Deployments
Runs the deployment 'crex_tests' (mars.orogen) along with the tasks 'trajectory_generation::Task', 'behavior_graph::MotionControlTask' and 'joint_dispatcher::Task'.

## gdal_to_mars
old.

## just_asguard.rb
### Behavior description
Asguard but no control or perception task running.
### Deployments
Runs the deployment 'just_mars' (mars.orogen).

## just_crex.rb
### Behavior description
Crex but no control or perception task running.
### Deployments
Runs the deployment 'just_mars' (mars.orogen).

## mars_to_envire
old

## scene.rb
old
