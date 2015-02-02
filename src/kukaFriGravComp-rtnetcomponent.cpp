// Filename:  kukaFriGravComp-rtnetcomponent.cpp
// Copyright: 2015 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaFriGravComp-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fstream>
#include <rtt/os/TimeService.hpp>

KukaFriGravCompRTNET::KukaFriGravCompRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    this->addPort("weight", oport_weight);
    this->addPort("ati_sensor", iport_ati_values);

    this->addOperation("setFThreshold", &KukaFriGravCompRTNET::setFThreshold, this, RTT::OwnThread);
    this->addOperation("setLoadThreshold", &KukaFriGravCompRTNET::setLoadThreshold, this, RTT::OwnThread);
    this->addOperation("setNumObsTau", &KukaFriGravCompRTNET::setNumObsTau, this, RTT::OwnThread);
    this->addOperation("setNumObsLoad", &KukaFriGravCompRTNET::setNumObsLoad, this, RTT::OwnThread);
    this->addOperation("setTau", &KukaFriGravCompRTNET::setTau, this, RTT::OwnThread);
    this->addOperation("setNumObsForce", &KukaFriGravCompRTNET::setNumObsForce, this, RTT::OwnThread);
    this->addOperation("setTrajectory", &KukaFriGravCompRTNET::setTrajectory, this, RTT::OwnThread);
    this->addOperation("setBlock", &KukaFriGravCompRTNET::setBlock, this, RTT::OwnThread);
    this->addOperation("setStiffness", &KukaFriGravCompRTNET::setStiffness, this, RTT::OwnThread);
    this->addOperation("s1", &KukaFriGravCompRTNET::setStiffness1, this, RTT::OwnThread);
    this->addOperation("s2", &KukaFriGravCompRTNET::setStiffness2, this, RTT::OwnThread);
    this->addOperation("connectPorts", &KukaFriGravCompRTNET::connectPorts, this, RTT::OwnThread);
    this->addOperation("dumpLog", &KukaFriGravCompRTNET::dumpLog, this, RTT::OwnThread);

	exitGravCompDelay = 2;
	startGravCompTime = 0;

    direction = 1;
    setNumObsTau(2000);
    setNumObsForce(40);
    setNumObsLoad(200);

	tau_cmd = 4.0;
	block = 0;
	trajectory = 0;
	gravComp = 0;

	iteration = 0;

	forceThreshold = 5.0;
	loadThreshold = 0.5;

    m_joint_vel_command.resize(LWRDOF);
    std::fill(m_joint_vel_command.begin(), m_joint_vel_command.end(), 0.0);

	m_joint_pos.resize(LWRDOF);
    std::fill(m_joint_pos.begin(), m_joint_pos.end(), 0.0);

    tau.resize(LWRDOF);
    std::fill(tau.begin(), tau.end(), 0.0);

    est_tau.resize(LWRDOF);
    std::fill(est_tau.begin(), est_tau.end(), 0.0);

	estExtTcpWrench.resize(3);
    std::fill(estExtTcpWrench.begin(), estExtTcpWrench.end(), 0.0);

	force_sensor_value.resize(8);
    std::fill(force_sensor_value.begin(), force_sensor_value.end(), 0.0);

}

void KukaFriGravCompRTNET::updateHook(){
   fri_frm_krl = m_fromKRL.get(); 

   RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(m_joint_pos);
   RTT::FlowStatus cart_pos_fs = iport_cart_pos.read(m_cart_pos);

   RTT::FlowStatus force_sensor_fs = iport_ati_values.read(force_sensor_value);

   double fz = force_sensor_value[2];
   double weight = fz / 9.80665;

   double normWeight = sqrt(weight*weight);

   std_msgs::Float64 weight_msg;
   weight_msg.data = normWeight;
   oport_weight.write(weight_msg);

   if(sqrt((current_load - normWeight) * (current_load - normWeight)) > loadThreshold){
	   setLoad(loadMean.getMean(normWeight+0.750));
   }
   else{
       setLoad(loadMean.getMean(0.750));
   }

   if(iport_est_ext_joint_trq.connected()){
	   RTT::FlowStatus est_tau_fs = iport_est_ext_joint_trq.read(est_tau);
	   log_est_tau.push_back(est_tau);
   }

   if(iport_cart_wrench.connected()){
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
		   //estExtTcpWrench[3] = wrench.torque.x;
		   //estExtTcpWrench[4] = wrench.torque.y;
		   //estExtTcpWrench[5] = wrench.torque.z;
		   log_estExtTcpWrench.push_back(estExtTcpWrench);
		   if(trajectory == 1){
			   if((extForceMean.getMean(v.Norm()) > forceThreshold)){
				   if(gravComp == 0){
					   //std::cout << "GravComp\n";
					   gravComp = 1;
					   startGravCompTime = RTT::os::TimeService::Instance()->getNSecs()/1e9;
				   }
			   }
			   else{
				   long int currentTime = RTT::os::TimeService::Instance()->getNSecs()/1e9;
				   if(gravComp == 1 && (currentTime - startGravCompTime) > 2){
					   //std::cout << "Traj\n";
					   gravComp = 0;
				   }
			   }
		   }
	   }
   }
   else{
	   estExtTcpWrench.assign(3, 0.0);

   }

   //if command mode
   if(fri_frm_krl.intData[0] == 1){ 
	   if((trajectory == 1) && (gravComp == 0)){
		   if(m_joint_pos[2] > 0.21975512 && direction == 1){
			   direction = -1;
		   }
		   else if( m_joint_pos[2] < -0.21975512 && direction == -1){
			   direction = 1;
		   }

		   tau[2] = tauMean.getMean( tau_cmd * direction );
	   }
	   else{
		   if((trajectory == 1) && (gravComp == 1)){
			   tau[2] = tauMean.getMean( 0.0 );
		   }
		   std::fill(tau.begin(), tau.end(), 0.0);
	   }

	   oport_add_joint_trq.write(tau);
	   log_tau.push_back(tau);
   }
   if(block == 0){
	   oport_joint_position.write(m_joint_pos);
   }

   iteration++;
}

void KukaFriGravCompRTNET::setTrajectory(int traj){
	trajectory = traj;
}

void KukaFriGravCompRTNET::setBlock(int value){
	block = value;
}

void KukaFriGravCompRTNET::setTau(double t){
	tau_cmd = t;
}

void KukaFriGravCompRTNET::setNumObsTau(unsigned int numObs){
    tauMean.setNumObs(numObs);
}

void KukaFriGravCompRTNET::setNumObsLoad(unsigned int numObs){
    loadMean.setNumObs(numObs);
}

void KukaFriGravCompRTNET::setNumObsForce(unsigned int numObs){
    extForceMean.setNumObs(numObs);
}

void KukaFriGravCompRTNET::connectPorts(){
	connectIEvents();
	connectIEstExtJntTrq();
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
			joint_impedance_command.stiffness[i] = 2000;
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
		std::cout << "Joint impedance " << s << std::endl;
    }

}

void KukaFriGravCompRTNET::setStiffness1(){
	setStiffness(250, 0.5);
}

void KukaFriGravCompRTNET::setStiffness2(){
	setStiffness(2000, 0.9);
}

void KukaFriGravCompRTNET::dumpLog(std::string filename, std::string filename2){
	std::ofstream of;
	of.open(filename.c_str());
	std::vector< std::vector<double> >::iterator it;
	for(it = log_estExtTcpWrench.begin(); it != log_estExtTcpWrench.end(); ++it){
		for(int i = 0; i < 3; ++i){
			of << (*it)[i] << " ";
		}
		of << std::sqrt((*it)[0]*(*it)[0] + (*it)[1]*(*it)[1] + (*it)[2]*(*it)[2]); 
		of << std::endl;
	}

	of.close();

	std::ofstream of2;
	of2.open(filename2.c_str());
	for(it = log_tau.begin(); it != log_tau.end(); ++it){
		for(int i = 0; i < 7; ++i){
			of2 << (*it)[i] << " ";
		}
		of2 << std::endl;
	}

	of2.close();

	std::ofstream of3;
	of3.open("est_tau");
	for(it = log_est_tau.begin(); it != log_est_tau.end(); ++it){
		for(int i = 0; i < 7; ++i){
			of3 << (*it)[i] << " ";
		}
		of3 << std::endl;
	}

	of3.close();

	return;
}

void KukaFriGravCompRTNET::setFThreshold(double t){
	forceThreshold = t;
}

void KukaFriGravCompRTNET::setLoadThreshold(double l){
	loadThreshold = l;
}

ORO_CREATE_COMPONENT(KukaFriGravCompRTNET)
