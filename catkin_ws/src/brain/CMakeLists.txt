cmake_minimum_required(VERSION 2.8.3)
project(brain)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        genmsg
        message_generation
        roslint
        opinau_msgs
        )


roslint_cpp()

## Declare a catkin package
catkin_package(
        CATKIN_DEPENDS message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(brain src/brain.cpp)
target_link_libraries(brain ${catkin_LIBRARIES})

add_executable(debug_purpose_listener src/debug_purpose_listener.cpp)
target_link_libraries(debug_purpose_listener ${catkin_LIBRARIES})

add_executable(key_input_node src/key_input_node.cpp)
target_link_libraries(key_input_node ${catkin_LIBRARIES})
#add_dependencies(key_input_node beginner_tutorials_generate_messages_cpp) # erase ?

add_executable(key_input_simple_node src/key_input_simple_node.cpp)
target_link_libraries(key_input_simple_node ${catkin_LIBRARIES})

add_executable(test_omsg_talker src/test_omsg_talker.cpp)
target_link_libraries(test_omsg_talker ${catkin_LIBRARIES})
