#!/bin/bash

# installation file for komodo2 over ROS Kinetic and ubuntu 16.04 #

GREEN_TXT='\e[0;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
NO_COLOR='\033[0m'
LOGS_FOLDER_PATH="${HOME}/catkin_ws/src/setup_logs"
INSTALL_HW_COMPS=false #deremine if this script will install komodo2 hardware drivers

# get package folder path. this must be executed before other commands
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR && cd ..
PKG_PATH=`pwd`

printf "${WHITE_TXT}\n***Installing Komodo2 ROS-Kinetic Package***\n${NO_COLOR}"

# check for hardware argument, and determine installation type #
if [ $# -eq 0 ]
then
    printf "${WHITE_TXT}\nNo arguments supplied, installing package for standalone PC... ${NO_COLOR}\n"
elif [ $# -eq 1 ]
then  
    if [ $1 = "hw" ]
    then
        printf "${WHITE_TXT}\nGot hardware argument. Installing package for Komodo2 hardware... ${NO_COLOR}\n"
        INSTALL_HW_COMPS=true
    else
        printf "${RED_TXT}\nInvalid argument. Use 'hw' argument to install this package for Komodo hardware, or use no arguments to install this package for a standalone PC ${NO_COLOR}\n"
        exit 1
    fi
else
    printf "${RED_TXT}\nToo many arguments. Only one argument named hw is allowed ${NO_COLOR}\n"
    exit 1
fi


# validate ros version #
printf "${WHITE_TXT}\nChecking ROS Version...\n${NO_COLOR}"

version=`rosversion -d`
if [ "$version" == "kinetic" ]; then
  printf "${GREEN_TXT}ROS version OK${NO_COLOR}\n"
else
  printf "${RED_TXT}Error: found ROS version ${version}, please install ROS Kinetic and try again${NO_COLOR}\n"
  exit 1
fi

# validate catkin_ws/src folder exist #
printf "${WHITE_TXT}\nChecking if catkin_ws/src folder exist...\n${NO_COLOR}"
cd ~ 
if [ ! -d "catkin_ws" ]; then
  printf "${RED_TXT}~/catkin_ws folder does not exist. Create workspace named catkin_ws and try again ${NO_COLOR}\n"
else
cd catkin_ws 
if [ ! -d "src" ]; then
  printf "${RED_TXT}~/catkin_ws/src folder does not exist. Create workspace named catkin_ws with src directory inside ${NO_COLOR}\n"
  exit 1
fi

fi
printf "${GREEN_TXT}found catkin_ws/src folder${NO_COLOR}\n"

printf "${WHITE_TXT}\nInstalling 3rd party packages...\n${NO_COLOR}"
# third party packages #
sudo apt-get update
sudo apt-get dist-upgrade 
sudo apt-get upgrade 

# exit and notify immediately if a command exits with a non-zero status
set -eb

sudo apt-get -y install ros-kinetic-controller-manager 
sudo apt-get -y install ros-kinetic-control-toolbox  
sudo apt-get -y install ros-kinetic-transmission-interface 
sudo apt-get -y install ros-kinetic-joint-limits-interface 
sudo apt-get -y install ros-kinetic-gazebo-ros-control 
sudo apt-get -y install ros-kinetic-ros-controllers 
sudo apt-get -y install ros-kinetic-ros-control 
sudo apt-get -y install ros-kinetic-moveit
sudo apt-get -y install ros-kinetic-moveit-ros-planning
sudo apt-get -y install ros-kinetic-moveit-ros-planning-interface
sudo apt-get -y install ros-kinetic-move-base
sudo apt-get -y install ros-kinetic-navigation
sudo apt-get -y install ros-kinetic-hector-slam
sudo apt-get -y install ros-kinetic-gmapping
sudo apt-get -y install ros-kinetic-twist-mux
sudo apt-get -y install ros-kinetic-pid
sudo apt-get -y install ros-kinetic-joy
sudo apt-get -y install ros-kinetic-usb-cam 
sudo apt-get -y install ros-kinetic-ar-track-alvar
sudo apt-get -y install joystick
sudo apt-get -y install ros-kinetic-hector-gazebo-plugins
sudo apt-get -y install ros-kinetic-serial
sudo apt-get -y install espeak espeak-data libespeak-dev
sudo apt-get -y install ros-kinetic-robot-localization
sudo apt-get -y install ros-kinetic-trac-ik ros-kinetic-moveit-kinematics 
sudo apt-get -y install ros-kinetic-urg-node 
sudo apt-get -y install ros-kinetic-rtabmap-ros
sudo apt-get -y install jstest-gtk
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# komodo2 ric interface #
printf "${WHITE_TXT}\nInstalling ric interface...\n${NO_COLOR}"
cd $PKG_PATH/komodo2/third_party_files/
sudo dpkg -i ros-kinetic-ric-interface_0.0.0-0xenial_amd64.deb
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# realsense depth camera #sudo /etc/init.d/udev restart
if [ "$INSTALL_HW_COMPS" = true ] ; then
    printf "${WHITE_TXT}\nInstalling depth camera...\n${NO_COLOR}"
    cd ~/catkin_ws/src/
    wget https://github.com/intel-ros/realsense/archive/2.0.3.tar.gz
    tar -xvzf 2.0.3.tar.gz
    rm 2.0.3.tar.gz     
    wget https://github.com/IntelRealSense/librealsense/archive/v2.10.3.tar.gz
    tar -xvzf v2.10.3.tar.gz
    rm v2.10.3.tar.gz
    sudo apt-get -y install libusb-1.0-0-dev pkg-config libgtk-3-dev
    sudo apt-get -y install libglfw3-dev                                                                                                                                                
    cd librealsense-2.10.3                                                                                                                                                               
    mkdir build && cd build               
    cmake ../  
    sudo make uninstall && make clean && make -j8 && sudo make install
    printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
fi

# usb rules #
if [ "$INSTALL_HW_COMPS" = true ] ; then
    printf "${WHITE_TXT}\nInstalling USB rules...\n${NO_COLOR}"
    sudo cp $PKG_PATH/komodo2/rules/roboteq.rules /etc/udev/rules.d
    sudo cp $PKG_PATH/komodo2/rules/49-teensy.rules /etc/udev/rules.d
    sudo cp $PKG_PATH/komodo2/rules/hokuyo.rules /etc/udev/rules.d/
    sudo cp $PKG_PATH/komodo2/rules/bms_battery.rules /etc/udev/rules.d
    sudo cp $PKG_PATH/komodo2/rules/99-realsense-libusb.rules /etc/udev/rules.d
    sudo cp $PKG_PATH/komodo2/rules/microsoft_webcam.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && udevadm trigger
    printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
fi

# compiling komodo2 #
printf "${WHITE_TXT}Compiling komodo2 package...\n${NO_COLOR}"
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

printf "${GREEN_TXT}Installation process finished.\n\n${NO_COLOR}"
printf "${GREEN_TXT}Please reboot to apply changes\n\n${NO_COLOR}"
exit 0
