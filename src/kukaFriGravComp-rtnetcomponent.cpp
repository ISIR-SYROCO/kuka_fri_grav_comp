// Filename:  kukaFriGravComp-rtnetcomponent.cpp
// Copyright: 2015 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaFriGravComp-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fstream>
#include <rtt/os/TimeService.hpp>
#include "Compute_C_limit.h"




KukaFriGravCompRTNET::KukaFriGravCompRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    this->addPort("weight", oport_weight);
    this->addPort("load", oport_load);
    this->addPort("normforce_robot", oport_normforce_robot_frame);
    this->addPort("ati_sensor", iport_ati_values);
    this->addPort("distance_port", iport_distance);
    this->addPort("distance_out", oport_distance);
    this->addPort("tau2_out", oport_tau2);
    this->addPort("res_C_out", oport_res_C);

    this->addOperation("setFThreshold", &KukaFriGravCompRTNET::setFThreshold, this, RTT::OwnThread);
    this->addOperation("setDefaultLoad", &KukaFriGravCompRTNET::setDefaultLoad, this, RTT::OwnThread);
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

    default_load = 0.770;

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

   RTT::FlowStatus distance_fs = iport_distance.read(distance_data);

   double fx = force_sensor_value[0];
   double fy = force_sensor_value[1];
   double fz = force_sensor_value[2];
   double weight = fz / 9.80665;

   double normWeight = sqrt(weight*weight);
   double normForce = sqrt(force_sensor_value[0]*force_sensor_value[0] / (9.80665*9.80665) + 
		   force_sensor_value[1]*force_sensor_value[1] / (9.80665*9.80665) +
		   force_sensor_value[2]*force_sensor_value[2] / (9.80665*9.80665));

   weight_msg.data = normWeight;
   oport_weight.write(weight_msg);

   force_norm_msg.data = normForce;
   oport_normforce_robot_frame.write(force_norm_msg);

   std_msgs::Float64 load_msg;
   double new_load = 0.0;
   if(sqrt((current_load - normForce) * (current_load - normForce)) > loadThreshold){
       if(normForce > loadThreshold){
           new_load = loadMean.getMean(normForce+default_load);
       }
       else{
           new_load = loadMean.getMean(default_load);
       }
       load_msg.data = new_load;
       setLoad(new_load);
   }
   oport_load.write(load_msg);

   if(iport_est_ext_joint_trq.connected()){
	   RTT::FlowStatus est_tau_fs = iport_est_ext_joint_trq.read(est_tau);
	   log_est_tau.push_back(est_tau);
   }

   if(iport_cart_wrench.connected()){
	   geometry_msgs::Wrench wrench;
	   RTT::FlowStatus est_ext_tcp_wrench_fs = iport_cart_wrench.read(wrench);
	   if (est_ext_tcp_wrench_fs == RTT::NewData){

		   KDL::Vector v(wrench.force.x, wrench.force.y, wrench.force.z);
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
				   if(gravComp == 1 && (currentTime - startGravCompTime) > 10){
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

   oport_distance.write(distance_data);
   double coeff_distance = distance_data.data/4;

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
		   
		   tau2_trace.data = tau[2];
		
       double nxt_step_des_C          = tau[2]; 
       double C_limit                 = compute_C_limit(1.5, 8, tau[2], 0.1, 2, distance_data.data);
       double coeff_C_limit           = C_limit/nxt_step_des_C;   
       double res_C          = coeff_C_limit * nxt_step_des_C;    
       
       res_C_trace.data = res_C;
       oport_res_C.write(res_C_trace);
       oport_tau2.write(tau2_trace);
       
       tau[2] = res_C;
       
       double kp = 2.0;
       double error_j0 = kp*(1.57 - m_joint_pos[0]);
       double error_j1 = kp*(0.35 - m_joint_pos[1]);
       double error_j3 = kp*(-1.78 - m_joint_pos[3]);
       double error_j4 = kp*(- m_joint_pos[4]);

		tau[0] = error_j0;
		tau[1] = error_j1;		
		tau[3] = error_j3;	
		tau[4] = error_j4;
		
	   }
	   else{
		   if((trajectory == 1) && (gravComp == 1)){
			   tau[2] = tauMean.getMean( 0.0 );
		   }
		   std::fill(tau.begin(), tau.end(), 0.0);
	   }
	   
	   
		
		// Ajout frottement visqueux
		Eigen::Matrix<double, 7, 1> q_dot; 			         //Joint velocities
		std::vector<double> q_dot_fri(7);
		RTT::FlowStatus joint_vel_fs = iport_msr_joint_vel.read(q_dot_fri);
        if(joint_vel_fs == RTT::NewData){    
        	for(unsigned int i = 0; i < 7; i++){
            	q_dot[i] = q_dot_fri[i];
            	tau[i] -= 10*q_dot_fri[i];
        	}
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

void KukaFriGravCompRTNET::setDefaultLoad(double l){
	default_load = l;
}

void KukaFriGravCompRTNET::setLoadThreshold(double l){
	loadThreshold = l;
}

ORO_CREATE_COMPONENT(KukaFriGravCompRTNET)
