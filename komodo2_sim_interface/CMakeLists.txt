cmake_minimum_required(VERSION 2.8.3)
project(komodo2_sim_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        gazebo_dev
        roscpp
        std_msgs
        control_toolbox
        controller_manager
        hardware_interface
        transmission_interface
        pluginlib
        joint_limits_interface
        urdf
        angles
        gazebo_ros_control
        )

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        controller_manager
        control_toolbox
        pluginlib
        hardware_interface
        transmission_interface
        joint_limits_interface
        urdf
        angles
        INCLUDE_DIRS include
        LIBRARIES komodo2_sim_interface
)

link_directories(
        ${catkin_LIBRARY_DIRS}
)

include_directories(include
        ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        )

## Libraries
#add_library(${PROJECT_NAME} src/gazebo_ros_control_plugin.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(komodo2_sim_interface src/komodo2_sim_interface.cpp include/komodo2_sim_interface/komodo2_sim_interface.h)
target_link_libraries(komodo2_sim_interface ${catkin_LIBRARIES})

## Install
install(TARGETS komodo2_sim_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )


install(FILES komodo2_sim_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )