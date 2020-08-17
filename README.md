 <!--:construction_worker: :construction: **_This page is under construction_** :construction: :construction_worker:-->
# Package for simulating wheel loader in gazebo engine

![komodo](https://i.imgur.com/ZCKDDNQ.png)

This page provides instructions and source code for simulating wheel loader and a pile of particles in gazebo. This is complementary material to the paper
> ***Wheel Loader Scooping Controller using Deep Reinforcement learning***
submitted to the *IEEE Access (2020)*.

We provide a platform in which the user can independently collect data on a simulator using the basic envs in ```environments``` folder.
Within ```/src/environments/komodo_env.py``` the user can change pile features (shape, amount, etc..) which defined in the ```Pile``` class. 
The user can also use ```/src/environments/komodo_env_new.py``` for diffrent pile arrangements on every episode (currently support up to 3 diffrent arrangements)



# Files 


```komodo2/launch/komodo2.launch``` : (parameters: depth_cam,lidar,gazebo): lauch all the required nodes.

```komodo2_control/config/komodo2_control.yaml``` : joint controller config for Gazebo

```komodo2_description/urdf/komodo2.xacro``` : urdf model in real robot

```komodo2_description/urdf/komodo2_gazebo.xacro``` : urdf model in Gazebo


```komodo2_rl/src/agents/ddpg.py``` : Deep Deterministic Policy Gradient agent (tensorflow)

```komodo2_rl/src/agents/a2c.py``` : Advantage actor-critic A2C agent (tensorflow)

```komodo2_rl/src/agents/utils.py``` : utilities (replay buffer, OUNoise


```komodo2_rl/src/environments/Spawner.py``` : object spawner script (can be edit to include other shapes)

```komodo2_rl/src/environments/komodo_env.py``` : environment setup in Gazebo

```komodo2_rl/src/environments/komodo_env_new.py``` : environment setup in Gazebo with random piles (3)


```komodo2_rl/src/komodo_learn.py``` : learning script ( change param ddpg/a2c)

```komodo2_rl/src/komodo_gazebo_act.py``` : learned agent acts in Gazebo

```komodo2_rl/src/komodo_model.py``` : enviorment setup real robot

```komodo2_rl/src/komodo_act.py``` : learned agent takes actions in a real robot

```komodo2_rl/src/komodo_maunal_control.py``` : manual control node using joy package



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

# How to

#### Training in Gazebo
```
roslaunch komodo2 komodo2.launch gazebo:=true
python komodo_learn.py
```

#### Learned model in gazebo
```
roslaunch komodo2 komodo2.launch gazebo:=true
python komodo_gazebo_act.py
```

#### learned agent in real robot
```
roslaunch arm_control arm.launch
rosrun arm_control inv_control.launch
roslaunch komodo2 komodo2.launch lidar:=true
python komodo_act.py
```

