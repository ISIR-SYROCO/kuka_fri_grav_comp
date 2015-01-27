// Filename:  kukaFriGravComp-rtnetcomponent.cpp
// Copyright: 2015 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaFriGravComp-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>

KukaFriGravCompRTNET::KukaFriGravCompRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    this->addOperation("setNumObs", &KukaFriGravCompRTNET::setNumObs, this, RTT::OwnThread);

    direction = 1;
    setNumObs(20);

    m_joint_vel_command.resize(LWRDOF);
    std::fill(m_joint_vel_command.begin(), m_joint_vel_command.end(), 0.0);
}

void KukaFriGravCompRTNET::updateHook(){
   fri_frm_krl = m_fromFRI.get(); 
   if(fri_frm_krl.intData[0] == 1){ //command mode

       std::vector<double> JState(LWRDOF);

       RTT::FlowStatus joint_state_fs =iport_msr_joint_pos.read(JState);

       if(joint_state_fs == RTT::NewData){
           if(JState[2] > 1.0471975512 && direction == 1){
               direction = -1;
           }
           else if( JState[2] < -1.0471975512 && direction == -1){
               direction = 1;
           }

           m_joint_vel_command[2] = mean.getMean( 0.2 * direction );

           oport_joint_velocities.write(m_joint_vel_command);

       }
   }

}

void KukaFriGravCompRTNET::setNumObs(unsigned int numObs){
    mean.setNumObs(numObs);
}

ORO_CREATE_COMPONENT(KukaFriGravCompRTNET)
