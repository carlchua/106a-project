## Dependencies:
[urdf2casadi](https://github.com/mahaarbo/urdf2casadi)

## Usage:
Replace the urdf file in unitree\_ros/a1\_description with the provided modified version

## Running the robot on Gazebo:
Add <node pkg="rostopic" type="rostopic" name="rostopic" args="pub /traj_opt/start_traj std_msgs/Bool 'data: True'"/> to the end of the launch file: unitree_ros/unitree_gazebo/launch/normal.launch

