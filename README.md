# Gravity Compensation


This is a repo for identification of robotic dynamic parameters including mass, inertia, friction, center of mass.

Currently, the code can only support revolute robots with fixed base. (Also theoretically support 6Dof robots).

The code has been tested on KUKA LBR MED R700 robot and Ubuntu 20.04 (ROS2 Version: Foxy).

The dependence of the libaray includes:



1. sudo apt install python3-pip
2. install optas refer to https://github.com/cmower/optas
3. pip3 install numpy==1.26.1
4. pip3 install open3d==0.17.0
5. pip3 install pybullet


install lbr_fri_ros2 refer to https://github.com/lbr-stack/lbr_fri_ros2_stack

install xarm_ros2 refer to https://github.com/xArm-Developer/xarm_ros2/tree/humble