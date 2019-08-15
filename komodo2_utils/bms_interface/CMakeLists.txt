cmake_minimum_required(VERSION 2.8.3)
project(bms_interface)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        roscpp std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bms_interface
        CATKIN_DEPENDS roscpp std_msgs
  #DEPENDS system_lib
)

set(
        PROJECT_FILES
        src/bms_interface.cpp
        include/bms_interface/bms_interface.h
        example/bms_example.cpp
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
         ${PROJECT_FILES}
   )


add_dependencies(${PROJECT_NAME} ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${${PROJECT_NAME}_gencpp} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node ${PROJECT_FILES})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

 install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )