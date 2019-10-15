### [Komodo Robot RL StartUp](#install-nvidia-drivers)


**NOTE**: This instruction was tested on `Ubuntu 16.04` and `ROS Kinetic Kame` and `Robitican Komodo2` .

1. start up ROS (#NotRequired)
``` bash
roscore
```

2. launch arm communication
``` bash
roslaunch arm_control arm.launch
```

3. launch rqt
``` bash
rqt
```

4. launch arm controller
``` bash
rosrun arm_control inv_control.py
```

5. run komodo_act / komodo_learn, depen on your request
