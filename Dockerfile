FROM ros:noetic

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rosserial-arduino ros-${ROS_DISTRO}-catkin \
ros-${ROS_DISTRO}-roslint qt5-default qtdeclarative5-dev

COPY arduino arduino
COPY catkin_ws/src catkin_ws/src

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd /catkin_ws && catkin_make

CMD ["rosrun", "rosserial_arduino", "make_libraries.py", "./arduino"]
