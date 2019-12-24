# DDPG Implementation on the komodo

# Files ( komodo2 folder)


- komodo2/launch/komodo2.launch : (parameters: depth_cam,lidar,gazebo): main launch 
- komodo2_control/config/komodo2_control.yaml : joint controller config for Gazebo
- komodo2_description/urdf/komodo2.xacro : urdf model in real robot
- komodo2_description/urdf/komodo2_gazebo.xacro : urdf model in Gazebo

- komodo2_rl/src/ddpg.py : Deep Deterministic Policy Gradient agent (tensorflow)
- komodo2_rl/src/Spawner.py : object spawner in gazebo script (Pile,particles etc)
- komodo2_rl/src/komodo_env.py : enviorment setup in Gazebo
- komodo2_rl/src/komodo_learn.py : ddpg learning script
- komodo2_rl/src/komodo_gazebo_act.py : learned agent acts in Gazebo

- komodo2_rl/src/komodo_model.py : enviorment setup real robot
- komodo2_rl/src/komodo_act.py : learned agent takes actions in a real robot

- komodo2_rl/src/replay_buffer_memory.p : agent replay buffer memory
- komodo2_rl/src/eps_rewards.npy : agent episode rewards
- komodo2_rl/src/plot_eps_rewards.py : plot episode rewards from eps_rewards.npy


# Prerequisites 
- python 2.7
- ROS Kinetic Kame
- Tensorflow >= 1.4
- Gazebo simulation

### additional installations for simulation
- ROS control
```
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-gazebo-ros-control
```
- Gazebo simulation update -> update to 7.x (7.0 causes error)


#### Learning an agent
```
roslaunch komodo2 komodo2.launch gazebo:=true
python komodo_learn.py
```

#### simulate learned agent
```
roslaunch komodo2 komodo2.launch gazebo:=true
python komodo_gazebo_act.py
```

#### learned agent on real quadruped

- on quadruped
```
roslaunch arm_control arm.launch
rosrun arm_control inv_control.launch
roslaunch komodo2 komodo2.launch lidar:=true
rqt
python komodo_act.py
```

# Tips
#### always
```
source ~/catkin_ws/devel/setup.bash
```
#### ROS core
roscore

#### ROS via network (local LAN)
- MASTER Setting -> get master_ip_addr via ifconfig (master runs roscore)
```
export ROS_MASTER_URI=http://master_ip_addr:11311
export ROS_IP=master_ip_addr
export ROS_MASTER_URI=http://192.168.0.5:11311
```

- SECOND Setting (from other computer)
```
ssh second_ip_addr -p(port_num)
export ROS_MASTER_URI=http://master_ip_addr:11311
export ROS_IP=second_ip_addr
```

- exiting ssh
```
exit
```

#### xacro to URDF file
```
rosrun xacro xacro --inorder model.xacro > model.urdf
```


