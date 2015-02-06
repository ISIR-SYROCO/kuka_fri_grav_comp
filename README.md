Kuka FRI Grav Comp
==================

This orocos component demonstrate a simple case of
automatic mode switching for the kuka robot through FRI.

In the default mode, the robot follow a simple trajectory.
The gravity compensation is activated when the operator touches the robot.
The ATI force sensor is used to adapt gravity compensation to the load of the robot .

Description
-----------

The component read a distance from the kinect module to scale the torque
sent to the robot.

The external Forces at Tool Center Point are read from the lwr_fri component,
if the norm of the force are above a threshold, then the gravity compensation is activated
(send null torques).

If the norm of the force read by the force sensor is above a certain threshold
then the component send the value of that norm in kg plus the default load to the 
KRL script through the shared array FRI_*_REA.
The KRL script update the system variable $LOAD.M
which represent the weight of the tool attached to the robot.
As a consequence, the robot compensate automatically the weight of the new load.


Install
-------

Standard procedure of ros node installation in groovy.

`wstool set kuka_fri_grav_comp https://github.com/ISIR-SYROCO/kuka_fri_grav_comp.git --git`
`rosmake kuka_fri_grav_comp`

Dependencies
------------

Those ros node are required:
 - kuka_component (isir branch)
 - rtt_rosnode
 - ATISensor
 - directKinematics
 - lbr4_state_publisher


