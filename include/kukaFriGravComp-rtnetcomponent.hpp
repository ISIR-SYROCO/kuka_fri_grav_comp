// Filename:  kukaFriGravComp-rtnetcomponent.hpp
// Copyright: 2015 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: Orocos component using RTNET to switch from 
// precomputed trajectory to gravity compensation
// when a user touch the robot

#ifndef kukaFriGravComp_RTNET_COMPONENT_HPP
#define kukaFriGravComp_RTNET_COMPONENT_HPP

#include <friRTNetExampleAbstract.hpp>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <movingMean.hpp>

class KukaFriGravCompRTNET : public FriRTNetExampleAbstract{
    public:
        KukaFriGravCompRTNET(std::string const& name);

		void setTrajectory(int traj);
		void setBlock(int value);
		void setStiffness(double s, double d);
		void setStiffness1();
		void setStiffness2();

        void updateHook();

        void setNumObsTau(unsigned int numObs);
        void setNumObsLoad(unsigned int numObs);
        void setTau(double t);
        void setDefaultLoad(double l);
        void setNumObsForce(unsigned int numObs);

		void initializeCommand();

		void connectPorts();

		void dumpLog(std::string filename, std::string filename2);
		void setFThreshold(double t);
		void setLoadThreshold(double l);

        std::vector<double> m_joint_vel_command;
		std::vector<double> m_joint_pos;
        std::vector<double> tau;
        std::vector<double> est_tau;
        std::vector< std::vector<double> > log_tau;
        std::vector< std::vector<double> > log_est_tau;
		std::vector<double> estExtTcpWrench;
		geometry_msgs::Pose m_cart_pos;

		std::vector< std::vector<double> > log_estExtTcpWrench;

        MovingMean tauMean;
        MovingMean extForceMean;
        MovingMean loadMean;

		double forceThreshold;
		double loadThreshold;

        int direction;

		int iteration;

		int block;

		int trajectory;
		int gravComp;

        double default_load;

		int exitGravCompDelay;

		long int startGravCompTime;

		RTT::InputPort< std::vector<double> > iport_ati_values;
		std::vector<double> force_sensor_value;

        std_msgs::Float32 distance_data;
        RTT::InputPort< std_msgs::Float32 > iport_distance;

		std_msgs::Float32 tau2_trace;
		std_msgs::Float32 res_C_trace;
		RTT::OutputPort<std_msgs::Float32> oport_distance;
		RTT::OutputPort<std_msgs::Float32> oport_tau2;
		RTT::OutputPort<std_msgs::Float32> oport_res_C;
		RTT::OutputPort<std_msgs::Float64> oport_weight;
		RTT::OutputPort<std_msgs::Float64> oport_load;
		RTT::OutputPort<std_msgs::Float64> oport_normforce_robot_frame;
	double tau_cmd;
   
   std_msgs::Float64 force_norm_msg;
   std_msgs::Float64 weight_msg;
};



#endif
