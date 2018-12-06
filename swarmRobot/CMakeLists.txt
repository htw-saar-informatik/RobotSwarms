cmake_minimum_required(VERSION 2.8.3)
project(SwarmRobot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        genmsg
        )

## Generate messages in the 'msg' folder
add_message_files(
        DIRECTORY msg FILES
        Num.msg
        Mission.msg
)

## Generate services in the 'srv' folder
add_service_files(
        DIRECTORY srv FILES
        AddTwoInts.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

## Thesis

add_executable(FlockRobot src/robot/robot_system/RobotPilot.h src/robot/nodes/FlockRobot.cpp src/robot/robot_system/Robot.h src/robot/flock/FlockControl.h src/various/Constants.h src/robot/flock/Statistics.h src/basics/Angle.h src/logger/Logger.h src/logger/Time.h src/objects/Object.h src/objects/Geometry.h src/basics/BasicStructs.h src/robot/robot_system/RobotCore.h src/basics/MyAlgorithms.h)
target_link_libraries(FlockRobot ${catkin_LIBRARIES})

add_executable(Observer src/robot/robot_system/RobotPilot.h src/robot/nodes/Observer.cpp src/robot/robot_system/Robot.h src/robot/flock/FlockControl.h src/various/Constants.h src/robot/flock/Statistics.h src/basics/Angle.h src/logger/Logger.h src/logger/Time.h src/objects/Object.h src/objects/Geometry.h src/basics/BasicStructs.h src/robot/robot_system/RobotCore.h src/basics/MyAlgorithms.h)
target_link_libraries(Observer ${catkin_LIBRARIES})

add_executable(Leader src/robot/robot_system/RobotPilot.h src/robot/nodes/Leader.cpp src/robot/robot_system/Robot.h src/robot/flock/FlockControl.h src/various/Constants.h src/robot/flock/Statistics.h src/basics/Angle.h src/logger/Logger.h src/logger/Time.h src/objects/Object.h src/objects/Geometry.h src/basics/BasicStructs.h src/robot/robot_system/RobotCore.h src/basics/MyAlgorithms.h)
target_link_libraries(Leader ${catkin_LIBRARIES})

add_executable(Statist src/robot/robot_system/RobotPilot.h src/robot/robot_system/Robot.h src/robot/flock/FlockControl.h src/various/Constants.h src/robot/flock/Statistics.h src/robot/nodes/Statist.cpp src/basics/Angle.h src/logger/Logger.h src/logger/Time.h src/objects/Object.h src/objects/Geometry.h src/basics/BasicStructs.h src/robot/robot_system/RobotCore.h src/basics/MyAlgorithms.h)
target_link_libraries(Statist ${catkin_LIBRARIES})

add_executable(TEST src/robot/robot_system/RobotPilot.h src/robot/robot_system/Robot.h src/robot/flock/FlockControl.h src/various/Constants.h src/robot/flock/Statistics.h src/TEST.cpp src/logger/Time.h src/objects/Object.h src/basics/BasicStructs.h src/robot/robot_system/RobotCore.h src/basics/MyAlgorithms.h)
target_link_libraries(TEST ${catkin_LIBRARIES})

set(COMPILE_OPTIONS -Wall -Weffc++ -pedantic
        -pedantic-errors -Wextra -Waggregate-return -Wcast-align
        -Wcast-qual -Wchar-subscripts -Wcomment -Wconversion
        -Wdisabled-optimization
        -Wfloat-equal -Wformat -Wformat=2
        -Wformat-security
        -Wformat-y2k
        -Wimplicit -Wimport -Winit-self -Winline
        -Winvalid-pch
        -Wunsafe-loop-optimizations -Wlong-long -Wmissing-braces
        -Wmissing-field-initializers -Wmissing-format-attribute
        -Wmissing-include-dirs -Wmissing-noreturn
        -Wpacked -Wpadded -Wparentheses -Wpointer-arith
        -Wredundant-decls -Wreturn-type
        -Wsequence-point -Wshadow -Wsign-compare -Wstack-protector
        -Wstrict-aliasing -Wstrict-aliasing=2 -Wswitch -Wswitch-default
        -Wswitch-enum -Wtrigraphs -Wuninitialized
        -Wunknown-pragmas -Wunreachable-code -Wunused
        -Wunused-function -Wunused-label -Wunused-parameter
        -Wunused-value -Wunused-variable -Wvariadic-macros
        -Wvolatile-register-var -Wwrite-strings
        )

target_compile_options(FlockRobot PRIVATE ${COMPILE_OPTIONS})
target_compile_options(Observer PRIVATE ${COMPILE_OPTIONS})
target_compile_options(Leader PRIVATE ${COMPILE_OPTIONS})
target_compile_options(Statist PRIVATE ${COMPILE_OPTIONS})
target_compile_options(TEST PRIVATE ${COMPILE_OPTIONS})

