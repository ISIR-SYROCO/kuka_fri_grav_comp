// Filename:  kukaFriGravComp-rtnetcomponent.cpp
// Copyright: 2015 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaFriGravComp-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fstream>

KukaFriGravCompRTNET::KukaFriGravCompRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    this->addOperation("setNumObs", &KukaFriGravCompRTNET::setNumObs, this, RTT::OwnThread);
    this->addOperation("setTrajectory", &KukaFriGravCompRTNET::setTrajectory, this, RTT::OwnThread);
    this->addOperation("setStiffness", &KukaFriGravCompRTNET::setStiffness, this, RTT::OwnThread);
    this->addOperation("s1", &KukaFriGravCompRTNET::setStiffness1, this, RTT::OwnThread);
    this->addOperation("s2", &KukaFriGravCompRTNET::setStiffness2, this, RTT::OwnThread);
    this->addOperation("connectPorts", &KukaFriGravCompRTNET::connectPorts, this, RTT::OwnThread);
    this->addOperation("dumpLog", &KukaFriGravCompRTNET::dumpLog, this, RTT::OwnThread);

    direction = 1;
    setNumObs(20);

	trajectory = 0;

	iteration = 0;

    m_joint_vel_command.resize(LWRDOF);
    std::fill(m_joint_vel_command.begin(), m_joint_vel_command.end(), 0.0);

	m_joint_pos.resize(LWRDOF);
    std::fill(m_joint_pos.begin(), m_joint_pos.end(), 0.0);

    tau.resize(LWRDOF);
    std::fill(tau.begin(), tau.end(), 0.0);

	estExtTcpWrench.resize(6);
    std::fill(estExtTcpWrench.begin(), estExtTcpWrench.end(), 0.0);

	log_estExtTcpWrench.resize(100);
}

void KukaFriGravCompRTNET::updateHook(){
   fri_frm_krl = m_fromFRI.get(); 

   RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(m_joint_pos);
   RTT::FlowStatus cart_pos_fs = iport_cart_pos.read(m_cart_pos);

   if(iport_cart_wrench.connected() && ((iteration % 1000) == 0)){
	   geometry_msgs::Wrench wrench;
	   RTT::FlowStatus est_ext_tcp_wrench_fs = iport_cart_wrench.read(wrench);
	   if (est_ext_tcp_wrench_fs == RTT::NewData){

		   KDL::Vector v(wrench.force.x, wrench.force.y, wrench.force.y);
		   KDL::Rotation cart_orientation = KDL::Rotation::Quaternion((double)m_cart_pos.orientation.x,
				   (double)m_cart_pos.orientation.y,
				   (double)m_cart_pos.orientation.z,
				   (double)m_cart_pos.orientation.w);
		   v = cart_orientation.Inverse() * v;

		   estExtTcpWrench[0] = v.x();
		   estExtTcpWrench[1] = v.y();
		   estExtTcpWrench[2] = v.z();
		   estExtTcpWrench[3] = wrench.torque.x;
		   estExtTcpWrench[4] = wrench.torque.y;
		   estExtTcpWrench[5] = wrench.torque.z;
		   log_estExtTcpWrench.push_back(estExtTcpWrench);
	   }
   }
   else{
	   estExtTcpWrench.assign(6, 0.0);

   }

   //if command mode
   if(fri_frm_krl.intData[0] == 1){ 
	   if(trajectory == 1){
		   if(m_joint_pos[2] > 1.0471975512 && direction == 1){
			   direction = -1;
		   }
		   else if( m_joint_pos[2] < -1.0471975512 && direction == -1){
			   direction = 1;
		   }

		   tau[2] = mean.getMean( 0.4 * direction );
	   }
	   else{
		   std::fill(tau.begin(), tau.end(), 0.0);
	   }

	   oport_add_joint_trq.write(tau);
   }

   if(joint_state_fs == RTT::NewData){
	   oport_joint_position.write(m_joint_pos);
   }

   iteration++;
}

void KukaFriGravCompRTNET::setTrajectory(int traj){
	trajectory = traj;
}

void KukaFriGravCompRTNET::setNumObs(unsigned int numObs){
    mean.setNumObs(numObs);
}

void KukaFriGravCompRTNET::connectPorts(){
    connectIMsrJntPos();
	connectODesJntImpedance();
	connectOJointPosition();
	connectOJointTorque();
	connectIEstExtTcpWrench();
	connectIMsrCartPos();
}

void KukaFriGravCompRTNET::initializeCommand(){
    //Get current joint position and set it as desired position
    if (oport_joint_position.connected()){
        std::vector<double> measured_jointPosition;
        RTT::FlowStatus measured_jointPosition_fs = iport_msr_joint_pos.read(measured_jointPosition);
        if (measured_jointPosition_fs == RTT::NewData){
            std::vector<double> joint_position_command_init;
            joint_position_command_init.assign(7, 0.0);
            for (unsigned int i = 0; i < LWRDOF; i++){
                joint_position_command_init[i] = measured_jointPosition[i];
            }
            oport_joint_position.write(joint_position_command_init);
        }
    }

    if(oport_cartesian_pose.connected()){
        //Get cartesian position and set it as desired position
        geometry_msgs::Pose cartPosData;
        RTT::FlowStatus cart_pos_fs = iport_cart_pos.read(cartPosData);
        if (cart_pos_fs == RTT::NewData){
            oport_cartesian_pose.write(cartPosData);
        }
    }

    if(oport_cartesian_wrench.connected()){
        //Send 0 torque and force
        geometry_msgs::Wrench cart_wrench_command;
        cart_wrench_command.force.x = 0.0;
        cart_wrench_command.force.y = 0.0;
        cart_wrench_command.force.z = 0.0;
        cart_wrench_command.torque.x = 0.0;
        cart_wrench_command.torque.y = 0.0;
        cart_wrench_command.torque.z = 0.0;

        oport_cartesian_wrench.write(cart_wrench_command);
    }

    if (oport_add_joint_trq.connected()){
        //Send 0 joint torque
        std::vector<double> joint_eff_command;
        joint_eff_command.assign(LWRDOF, 0.0);
        oport_add_joint_trq.write(joint_eff_command);
    }

    if (oport_joint_impedance.connected()){
        lwr_fri::FriJointImpedance joint_impedance_command;
		for(unsigned int i = 0; i < LWRDOF; i++){
			joint_impedance_command.stiffness[i] = 1700;
			joint_impedance_command.damping[i] = 0.9;
		}

		oport_joint_impedance.write(joint_impedance_command);
    }
}

void KukaFriGravCompRTNET::setStiffness(double s, double d){
    if (oport_joint_impedance.connected()){
        lwr_fri::FriJointImpedance joint_impedance_command;
		for(unsigned int i = 0; i < LWRDOF; i++){
			joint_impedance_command.stiffness[i] = s;
			joint_impedance_command.damping[i] = d;
		}

		oport_joint_impedance.write(joint_impedance_command);
    }

}

void KukaFriGravCompRTNET::setStiffness1(){
	setStiffness(250, 0.5);
}

void KukaFriGravCompRTNET::setStiffness2(){
	setStiffness(17000, 0.9);
}

void KukaFriGravCompRTNET::dumpLog(std::string filename){
	std::ofstream of;
	of.open(filename.c_str());
	std::vector< std::vector<double> >::iterator it;
	for(it = log_estExtTcpWrench.begin(); it != log_estExtTcpWrench.end(); ++it){
		for(int i = 0; i < 6; ++i){
			of << (*it)[i] << " ";
		}
		of << std::endl;
	}

	of.close();
	return;
}

ORO_CREATE_COMPONENT(KukaFriGravCompRTNET)
