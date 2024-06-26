# Gravity Compensation


This is a repo for identification of robotic dynamic parameters including mass, inertia, friction, center of mass.

Currently, the code can only support revolute robots with fixed base. (Also theoretically support 6Dof robots).

The code has been tested on KUKA LBR MED R700 robot and Ubuntu 20.04 (ROS2 Version: Foxy).

The dependence of the libaray includes:

sudo apt install python3-pip


# How to set alias for training environment

paste this in  ~/.bashconda3

'''
# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/pmulrooney/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/pmulrooney/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/home/pmulrooney/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/pmulrooney/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<
'''

paste this in .bashrc 

source /opt/ros/humble/setup.bash
alias conda_source='source ~/.bashconda3'



install optas refer to https://github.com/cmower/optas

Note numpy==1.26.1


install open3d==0.17.0
1. optas
2. lbr_fri_ros2
3. scipy
4. open3d
5. pybullet