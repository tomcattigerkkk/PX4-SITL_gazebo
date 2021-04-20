/*
 * @Author: Wei Luo
 * @Date: 2020-11-26 16:37:50
 * @LastEditors: Wei Luo
 * @LastEditTime: 2020-11-28 01:17:57
 * @Note: Note
 */

#ifndef _GAZEBO_TWO_ARM_CONTROLLER_PLUGIN_
#define _GAZEBO_TWO_ARM_CONTROLLER_PLUGIN_

#include <string>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh> // message transport
#include <gazebo/msgs/msgs.hh>


#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <std_msgs/Float32.h>

namespace gazebo{
	class GAZEBO_VISIBLE TwoArmControllerPlugin: public ModelPlugin
	{
		public:
			TwoArmControllerPlugin();
			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
			virtual void Init();

		private:
			void OnUpdate();
			// void GroundTruthCallback(GtPtr& imu_message);
			// msg callback
			#if GAZEBO_MAJOR_VERSION > 7 || (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION >= 4)
				// void OnArm1Msg(ConstAnyPtr &_msg);
				// void OnArm1Msg(const std_msgs::Float32ConstPtr &_msg);
				// void OnArm1Msg(const double* &_msg);
				// void OnArm2Msg(ConstAnyPtr &_msg);
			#else
			// void OnArm1Msg(ConstGzStringPtr &_msg);
			#endif
			void OnArm1Msg(const std_msgs::Float32ConstPtr &_msg);
			void OnArm2Msg(const std_msgs::Float32ConstPtr &_msg);
			void QueueThread();

			// parameters
			std::string status;
			physics::ModelPtr model;
			sdf::ElementPtr sdf;
			physics::JointPtr arm_joint1; // first arm
			physics::JointPtr arm_joint2; // second arm
			common::PID pid_arm1; // PID controller
			common::PID pid_arm2; // PID controller

			// msg mechanism
			transport::NodePtr node;
			transport::SubscriberPtr arm1Sub;
			transport::SubscriberPtr arm2Sub;
			transport::PublisherPtr arm1Pub;
			transport::PublisherPtr arm2Pub;

			// timer
			common::Time lastUpdateTime;

			// mutex
			std::mutex cmd_mutex;

			// command
			double arm1_cmd;
			double arm2_cmd;

			// connection
			std::vector<event::ConnectionPtr> connections;

			// ros
			std::unique_ptr<ros::NodeHandle> rosNode;
            ros::Subscriber rosArm1Sub;
            ros::Subscriber rosArm2Sub;
            ros::CallbackQueue rosQueue;
            std::thread rosQueueThread;

	};



}

#endif
