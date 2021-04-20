/*
 * @Author: Wei Luo
 * @Date: 2020-11-26 16:45:57
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-03-04 14:44:00
 * @Note: Note
 */

#include <gazebo_two_arm_controller_plugin.h>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(TwoArmControllerPlugin)

TwoArmControllerPlugin::TwoArmControllerPlugin()
	:status("closed")
	{

	}

void TwoArmControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	double arm1_rad;
	double arm2_rad;
	// Store the model pointer for convenience.
	this->model = _model;
	if (_sdf->HasElement("joint_1") && _sdf->HasElement("joint_2"))
		cout<<"Joint 1 and 2 are received"<<endl;
	else
		cout<<"Please specify both joints"<<endl;
	if (_sdf->HasElement("arm1_rad"))
	{
		arm1_rad = _sdf->Get<double>("arm1_rad");
	}
	else
		arm1_rad = 0.0;
	if (_sdf->HasElement("arm2_rad"))
	{
		arm2_rad = _sdf->Get<double>("arm2_rad");
	}
	else
		arm2_rad = 0.0;


	this->pid_arm1 = common::PID(0.1, 0.0, 0.02);
	this->pid_arm2 = common::PID(0.1, 0.0, 0.02);
	this->arm_joint1 = _model->GetJoints()[1]; // itm_s500_two_d_arm::pendulum_joint
	this->arm_joint2 = _model->GetJoints()[2]; // itm_s500_two_d_arm::pendulum2_joint
	this->model->GetJointController()->SetPositionPID(this->arm_joint1->GetScopedName(), this->pid_arm1);
	this->model->GetJointController()->SetPositionTarget(this->arm_joint1->GetScopedName(), arm1_rad);
	this->model->GetJointController()->SetPositionPID(this->arm_joint2->GetScopedName(), this->pid_arm2);
	this->model->GetJointController()->SetPositionTarget(this->arm_joint2->GetScopedName(), arm2_rad);

	if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
        }
	// create note
    this->rosNode.reset(new ros::NodeHandle("gazebo_arm_client"));



}

void TwoArmControllerPlugin::Init(){
	// init function will be called after the Load() function
	this->node = transport::NodePtr(new transport::Node());
	#if GAZEBO_MAJOR_VERSION >= 9
		this->node->Init(this->model->GetWorld()->Name());
		this->lastUpdateTime = this->model->GetWorld()->SimTime();
	#else
		this->node->Init(this->model->GetWorld()->GetName());
		this->lastUpdateTime = this->model->GetWorld()->GetSimTime();
	#endif

	// // arm 1 command via gz transport
	// std::string arm1Topic = std::string("~/") + this->model->GetName() + "/arm1_cmd";
	// this->arm1Sub = this->node->Subscribe(arm1Topic, &TwoArmControllerPlugin::OnArm1Msg, this);

	// // arm 2 command via gz transport
	// std::string arm2Topic = std::string("~/") + this->model->GetName() + "/arm2_cmd";
	// this->arm2Sub = this->node->Subscribe(arm2Topic, &TwoArmControllerPlugin::OnArm2Msg, this);

	// plugin update
	this->connections.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&TwoArmControllerPlugin::OnUpdate, this)));

	// // publish arm status
	// arm1Topic = std::string("~/")+this->model->GetName()+"/arm1_status";
	// this->arm1Pub = node->Advertise<gazebo::msgs::GzString>(arm1Topic);
	// arm2Topic = std::string("~/")+this->model->GetName()+"/arm2_status";
	// this->arm2Pub = node->Advertise<gazebo::msgs::GzString>(arm2Topic);

	ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
              "/" + this->model->GetName() + "/arm_1",
              1,
              boost::bind(&TwoArmControllerPlugin::OnArm1Msg, this, _1),
              ros::VoidPtr(), &this->rosQueue);

	ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Float32>(
              "/" + this->model->GetName() + "/arm_2",
              1,
              boost::bind(&TwoArmControllerPlugin::OnArm2Msg, this, _1),
              ros::VoidPtr(), &this->rosQueue);

	// subscribe
    this->rosArm1Sub = this->rosNode->subscribe(so);
    this->rosArm2Sub = this->rosNode->subscribe(so2);
    // ROS队列辅助线程
    this->rosQueueThread = std::thread(std::bind(&TwoArmControllerPlugin::QueueThread, this));

	gzmsg << "Initial finished"<<endl;
}

void TwoArmControllerPlugin::OnUpdate(){
	const std::lock_guard<std::mutex> lock(cmd_mutex);
	#if GAZEBO_MAJOR_VERSION >= 9
		common::Time time = this->model->GetWorld()->SimTime();
	#else
		common::Time time = this->model->GetWorld()->GetSimTime();
	#endif
	if (time < this->lastUpdateTime)
	{
		gzerr << "time reset event\n";
		this->lastUpdateTime = time;
		return;
	}
	else if (time > this->lastUpdateTime)
	{
		this->lastUpdateTime = time;
		this->model->GetJointController()->SetPositionTarget(this->arm_joint1->GetScopedName(), this->arm1_cmd);
		this->model->GetJointController()->SetPositionTarget(this->arm_joint2->GetScopedName(), this->arm2_cmd);
	}
}

// void TwoArmControllerPlugin::OnArm1Msg(ConstAnyPtr &_msg){
// 	const std::lock_guard<std::mutex> lock(cmd_mutex);
// 	this->arm1_cmd = _msg->double_value();
// }

void TwoArmControllerPlugin::OnArm1Msg(const std_msgs::Float32ConstPtr &_msg){
	const std::lock_guard<std::mutex> lock(cmd_mutex);
	this->arm1_cmd = _msg->data;
}

void TwoArmControllerPlugin::OnArm2Msg(const std_msgs::Float32ConstPtr &_msg){
	const std::lock_guard<std::mutex> lock(cmd_mutex);
	this->arm2_cmd = _msg->data;
}
// void TwoArmControllerPlugin::OnArm2Msg(ConstAnyPtr &_msg){
// 	const std::lock_guard<std::mutex> lock(cmd_mutex);
// 	this->arm2_cmd = _msg->double_value();
// }

void TwoArmControllerPlugin::QueueThread(){
    static const double timeout = 0.01;
    while (this->rosNode->ok())
   		this->rosQueue.callAvailable(ros::WallDuration(timeout));
}
// void TwoArmControllerPlugin::GroundTruthCallback(GtPtr& imu_message)
// {

// }
