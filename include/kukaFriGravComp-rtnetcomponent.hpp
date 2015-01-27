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

#include <movingMean.hpp>

class KukaFriGravCompRTNET : public FriRTNetExampleAbstract{
    public:
        KukaFriGravCompRTNET(std::string const& name);

		void setTrajectory(int traj);
		void setStiffness(double s, double d);

        void updateHook();

        void setNumObs(unsigned int numObs);

		void initializeCommand();

		void connectPorts();

		void dumpLog(std::string filename);

        std::vector<double> m_joint_vel_command;
		std::vector<double> m_joint_pos;
        std::vector<double> tau;
		std::vector<double> estExtTcpWrench;

		std::vector< std::vector<double> > log_estExtTcpWrench;

        MovingMean mean;
        int direction;

		int iteration;

		int trajectory;
};



#endif
