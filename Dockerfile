FROM ros:noetic

RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-rosserial-arduino

COPY arduino arduino
COPY catkin_ws/src catkin_ws/src

CMD ["rosrun", "rosserial_arduino", "make_libraries.py", "./arduino"]
