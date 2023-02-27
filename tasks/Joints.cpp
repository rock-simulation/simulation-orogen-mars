/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Joints.hpp"
#include <boost/foreach.hpp>
#include <mars_interfaces/sim/MotorManagerInterface.h>
#include <base/Logging.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <mars_interfaces/sim/ControlCenter.h>

using namespace mars;

// TODO: we should get it from simulator
#define SIM_CENTER_FRAME_NAME "world"

// TODO: add passive joint, without motors
// TODO: check effort, it seems to be always zero

Joints::Joints(std::string const& name)
    : JointsBase(name)
{
}

Joints::Joints(std::string const& name, RTT::ExecutionEngine* engine)
    : JointsBase(name, engine)
{
    //controlMode = mars::IGNORE;
}

Joints::~Joints()
{
}

void Joints::findJoints(const VertexDesc &vertex)
{
    std::cout << "FRAME: " << control->envireGraph->getFrameId(vertex) << std::endl;

    // parse the sub graph to find all required joints and their corresponding motors
    if(control->graphTreeView->tree.find(vertex) != control->graphTreeView->tree.end())
    {
        const std::unordered_set<VertexDesc>& children = control->graphTreeView->tree[vertex].children;
        // TODO: there is some issue if children is empty
        if (!children.empty()) {
            for(const VertexDesc child : children)
            {
                std::cout << "- child: " << control->envireGraph->getFrameId(child) << std::endl;
                using JointItem = envire::core::Item<interfaces::JointInterfaceItem>;
                using JointItemItr = envire::core::EnvireGraph::ItemIterator<JointItem>;
                JointItemItr begin_joint, end_joint, cur_joint;
                boost::tie(begin_joint, end_joint) = control->envireGraph->getItems<JointItem>(child);

                using SimMotorItem = envire::core::Item<std::shared_ptr<core::SimMotor>>;
                using SimMotorItemItr = envire::core::EnvireGraph::ItemIterator<SimMotorItem>;
                SimMotorItemItr begin_motor, end_motor, cur_motor;
                boost::tie(begin_motor, end_motor) = control->envireGraph->getItems<SimMotorItem>(child);

                // parse the joints of the frame to find the joints required by config
                for(cur_joint = begin_joint; cur_joint!=end_joint; cur_joint++)
                {
                    // all joint names saved with the prefix (robot name) inside the mars
                    std::shared_ptr<interfaces::JointInterface> jointInterface = cur_joint->getData().jointInterface;
                    std::string jointName;
                    jointInterface->getName(&jointName);

                    std::cout << "--- Joint: " << jointName << std::endl;

                    // check if the joint is required by config
                    if (std::find(jointNames.begin(), jointNames.end(), jointName) != std::end(jointNames))
                    {
                        std::cout << "------ found joint name" << std::endl;
                        // check if there is a motor for joint, than store the motor joint relation
                        // if not, store joint as passive
                        bool hasMotor = false;
                        cur_motor = begin_motor;
                        while (cur_motor != end_motor && hasMotor == false) {

                            std::shared_ptr<core::SimMotor> simMotor = cur_motor->getData();
                            std::cout << "simMotor: " << simMotor->getName() << " " << simMotor->getJointName() << std::endl;
                            // TODO: this is a temporary fix to match names of joints and joint names in the simMotor
                            // since later joint and motor are stored in their own frame
                            if (std::string(prefix + simMotor->getJointName()) == jointName)
                            {
                                std::cout << "motor: " << simMotor->getName() << std::endl;
                                motorJoints[simMotor->getName()] = std::make_pair(simMotor, jointInterface);
                                hasMotor = true;
                            }
                            cur_motor++;
                        }

                        if (hasMotor == false)
                        {
                            std::cout << "joint: passive" << std::endl;
                            passiveJoint.push_back(jointInterface);
                        }
                    }
                }
                std::cout << "call findjoints" << std::endl;
                findJoints(child);
            }
        }
    } else {
        LOG_ERROR_S << "SubWorld " << "SubWorld::" << prefix << " can not be found";
    }
}

void Joints::init()
{
}

void Joints::update(double delta_t)
{
    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    while (_command.read(jointCommand) == RTT::NewData)
    {
        for (auto &jointName : jointCommand.names)
        {
            if (motorJoints.count(jointName)) {
                base::JointState state = jointCommand.getElementByName(jointName);

                if( state.hasPosition() )
                {
                    motorJoints[jointName].first->setValue(state.position);
                }
                else
                {
                    if( state.hasSpeed() )
                        motorJoints[jointName].first->setVelocity(state.speed);
                }
                if( state.hasEffort() )
                {
                    LOG_WARN_S << "Effort command ignored for the joint '" << jointName << "'";
                }
                if( state.hasRaw() )
                {
                    LOG_WARN_S << "Raw command ignored for the joint '" << jointName << "'";
                }
            } else {
                LOG_ERROR_S << "There is no joint with the name '" << jointName << "' or this joint is passive";
            }

        }
    }

    // send the current state of the joints
    for (auto &jointName : jointStatus.names)
    {
        base::JointState state;
        // check if the joint has motor or passive
        // get the data from mars
        if (motorJoints.count(jointName)) {
            MJPair &mj = motorJoints[jointName];
            state.position = mj.first->getActualPosition();
            state.speed = mj.second->getVelocity();
            // TODO: should we use getMotorTorque?
            //state.effort = mj.first->getTorque();
            state.effort = mj.second->getMotorTorque();

            jointStatus[jointName] = state;
        }
        else {
            // TODO add passive joint
        }
    }
    jointStatus.time = getTime();
    _status.write(jointStatus);




    /*if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
    // if there was a command, write it to the mars
    while( _command.read( cmd ) == RTT::NewData )
    {
	for( size_t i=0; i<mars_ids.size(); ++i )
	{
        //passive joints can't take commands
	    if (joint_types [i] == PASSIVE){
	        continue;
	    }

	    // for each command input look up the name in the mars_ids structure
	    JointConversion conv = mars_ids[i];

            //ignore the case that the input data stream has not commands for our other joints
            std::vector<std::string>::const_iterator it = std::find(cmd.names.begin(), cmd.names.end(), conv.externalName);
            if (it == cmd.names.end()){
                continue;
            }

            base::JointState &curCmd(cmd[*it]);

	    mars::sim::SimMotor *motor = control->motors->getSimMotor( conv.mars_id );

	    if( curCmd.hasPosition() )
            {
                //set maximum speed that is allowed for turning

                if(curCmd.hasSpeed()){
                    switch (controlMode){
                    case IGNORE:break;
                    case MAX_SPEED:motor->setMaximumVelocity(curCmd.speed);break;
                    //case SPEED_AT_POS: RTT::log(RTT::Error) << "SPEED_AT_POS" << RTT::endlog();break
                    }
	            }
                motor->setValue( conv.toMars( curCmd.position ) );
            }
            else
            {
                if( curCmd.hasSpeed() )
                    motor->setVelocity(curCmd.speed / conv.scaling);
            }
	    if( curCmd.hasEffort() )
	    {
		LOG_WARN_S << "Effort command ignored";
	    }
	    if( curCmd.hasRaw() )
	    {
		LOG_WARN_S << "Raw command ignored";
	    }
	}
    }

    // in any case read out the status
    for( size_t i=0; i<mars_ids.size(); ++i )
    {
        JointConversion *conv = NULL;

        if (parallel_kinematics.empty()){
            conv = &(mars_ids[i]);
        }else{
            //mars_id does not fit the index of status,
            //find the conv by status name
            for (std::vector<JointConversion>::iterator it = mars_ids.begin();it != mars_ids.end();it++){
                if (it->externalName == status.names[i]){
                    conv = &(*it);
                    break;
                }
            }

        }

        base::JointState state;

        if (joint_types[i] == MOTOR){
            mars::sim::SimMotor *motor = control->motors->getSimMotor( conv->mars_id );

            state.position = conv->fromMars(conv->updateAbsolutePosition( motor->getActualPosition() ));
            state.speed = motor->getJoint()->getVelocity() * conv->scaling;
            state.effort = conv->fromMars( motor->getTorque() );

            currents[conv->externalName] = motor->getCurrent();

            status[conv->externalName] = state;
        }else{
            std::shared_ptr<mars::sim::SimJoint> joint = control->joints->getSimJoint( conv->mars_id );

            state.position = conv->fromMars(conv->updateAbsolutePosition( joint->getPosition() ));
            state.speed = joint->getVelocity() * conv->scaling;
            state.effort = 0;

            currents[conv->externalName] = 0;
            status[conv->externalName] = state;

        }
    }

    // and write it to the output port
    status.time = getTime();
    _status_samples.write( status );

    currents.time = status.time;
    _current_values.write(currents);

    // see if we have configuration for the joint_transforms
    // and the output port for it is connected
    std::vector<base::samples::RigidBodyState> rbs;
    if( !_joint_transform.value().empty() && _transforms.connected() )
    {
        _joint_transform.value().setRigidBodyStates( status, rbs );
        for( size_t i=0; i < rbs.size(); ++i )
            _transforms.write( rbs[i] );
    }*/
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Joints.hpp for more detailed
// documentation about them.

bool Joints::configureHook()
{
    // set robot name as prefix
    if (_robot_name.value() != "")
        prefix = _robot_name;

    // add prefix to the joint names from config to be able to find the joints inside the mars
    // since all mars elements contains prefix in their names
    jointNames.clear();
    jointNames = _names.value();
    std::for_each(jointNames.begin(), jointNames.end(),
        [&](std::string &jointName)
        { jointName = prefix + jointName; });

    std::cout << "PRINT JOINT NAMES" << std::endl;
    for (auto &jointName : jointNames)
    {
        std::cout << jointName << std::endl;
    }

    // TODO: for now we get SubWorld frame by its frame name, it can be changed later
    // find all joints that required by config
    const VertexDesc subWorldVertex = control->envireGraph->vertex("SubWorld::" + prefix);
    findJoints(subWorldVertex);

    // initialise jointStatus vector with the name of required joints
    size_t num_joints = jointNames.size();
    jointStatus.resize(num_joints);
    jointStatus.names = jointNames;

    /*size_t num_joints = _names.value().size();

    // test if scaling is valid
    if( !_scaling.value().empty() && _scaling.value().size() != num_joints )
    {
	LOG_ERROR_S << "The scaling property needs to be empty or of the same size as names.";
	return false;
    }
    if( !_offset.value().empty() && _offset.value().size() != num_joints )
    {
	LOG_ERROR_S << "The offset property needs to be empty or of the same size as names.";
	return false;
    }

    std::vector< mars::ParallelKinematic > parallel_kinematics = _parallel_kinematics.value();


    mars_ids.clear();
    // fill the joint structure
    mars_ids.resize( num_joints );
    cmd.resize( num_joints);
    status.resize( num_joints - parallel_kinematics.size() );


    currents.resize(num_joints - parallel_kinematics.size());


    std::vector<std::string> marsNames = _names.value();



    //set proper status names (by parallel kinematics)
    //status.resize( num_joints - parallel_kinematics.size());

    //set names
    if (parallel_kinematics.empty()){
    	status.names = _names.value();
    	currents.names = _names.value();
    }else{
    	printf("parallel kinematic_configuration:\n");
	    for (std::vector< mars::ParallelKinematic >::iterator it = parallel_kinematics.begin();it != parallel_kinematics.end();it++){
	    	printf("%s -> %s, %s\n",it->externalName.c_str(),it->internalName1.c_str(),it->internalName2.c_str());
	    }
	    std::vector<std::string> externalNames;
	    //set names for status
	    bool is_parallel = false;
	    for (std::vector<std::string>::iterator name = marsNames.begin(); name != marsNames.end();name++){
	    	is_parallel = false;
	    	for (std::vector< mars::ParallelKinematic >::iterator parallel = parallel_kinematics.begin();parallel != parallel_kinematics.end();parallel++){
	    		if (*name == parallel->internalName1){
	    			externalNames.push_back(parallel->externalName);
	    			is_parallel = true;
	    		}else if(*name == parallel->internalName2){
	    			//second is NOT used for status data
	    			is_parallel = true;
	    		}
	    	}
	    	if (!is_parallel){
	    		externalNames.push_back(*name);
	    	}
	    }
	    status.names = externalNames;
	    currents.names = externalNames;

    }


    std::vector<std::string> rename = _name_remap.get();
    for( size_t i=0; i<mars_ids.size(); i++ )
    {
		if( !_scaling.value().empty() ){
			mars_ids[i].scaling = _scaling.value()[i];
		}

		if( !_offset.value().empty() ){
			mars_ids[i].offset = _offset.value()[i];
		}
        mars_ids[i].marsName = marsNames[i];

		if(rename.empty() || rename[i].empty())
		{
			mars_ids[i].externalName = marsNames[i];

		}else
		{
			status.names[i] = rename[i];
            currents.names[i] = rename[i];
			mars_ids[i].externalName = rename[i];
		}

        if (!parallel_kinematics.empty()){
    	    //if (marsNames[i] in _parallel_kinematics )
    	    for (std::vector< mars::ParallelKinematic >::iterator it = parallel_kinematics.begin();it != parallel_kinematics.end();it++){
    	    	if (mars_ids[i].marsName == it->internalName1 || mars_ids[i].marsName == it->internalName2){
    	    		mars_ids[i].externalName = it->externalName;
    	    	}
    	    }
        }

    }

    controlMode = _controlMode.value();

*/
    //this needs to be called here, or we get a race condition
    //between the init of the plugin and the filling of mars_ids
    return JointsBase::configureHook();
}
bool Joints::startHook()
{
    if (! JointsBase::startHook())
        return false;
    return true;
}
void Joints::updateHook()
{
    JointsBase::updateHook();
}
void Joints::errorHook()
{
    JointsBase::errorHook();
}
void Joints::stopHook()
{
    JointsBase::stopHook();
}
void Joints::cleanupHook()
{
    JointsBase::cleanupHook();
}
