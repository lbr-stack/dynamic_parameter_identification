# Gravity Compensation

## Build
To build this repository, do
```shell
mkdir -p gravity_compensation_ws/src && cd gravity_compensation_ws && \ 
git clone https://github.com/mhubii/gravity_compensation.git src/gravity_compensation && \
colcon build
```

## Trajectory Acquisition
Executes a trajectory and records positions/velocities/efforts ie torques.
- Reads target joint positions from [target_joint_positions.csv](trajectory_acquisition/config/target_joint_positions.csv)
- Executes each configuration via [go_to](https://github.com/mhubii/gravity_compensation/blob/0b83c697d5eb2e5fa9d35ea25ec1220b579d5124/trajectory_acquisition/trajectory_acquisition/trajectory_execution_node.py#L157)
- Saves results as .csv via [save](https://github.com/mhubii/gravity_compensation/blob/0b83c697d5eb2e5fa9d35ea25ec1220b579d5124/trajectory_acquisition/trajectory_acquisition/trajectory_execution_node.py#L164)

To run, do
```shell
ros2 launch lbr_bringup lbr_bringup.launch.py model:=med7 sim:=false
```
In another terminal, run
```shell
ros2 launch trajectory_acquisition trajectory_acqusition.launch.py
```
