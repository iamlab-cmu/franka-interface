FROM ubuntu:16.04

RUN apt-get update

# Install ROS
COPY .docker_tmp/docker_util_scripts/install_ros.sh /
RUN bash /install_ros.sh
RUN rm /install_ros.sh

# # Install packages
RUN apt-get install -y git iputils-ping

RUN apt-get install -y cmake build-essential
RUN apt-get install -y libeigen3-dev libboost-all-dev
RUN apt-get install -y libpoco-doc libpoco-dev

ARG GITHUB_TOKEN
RUN git clone --recurse-submodules https://$GITHUB_TOKEN:@github.com/iamlab-cmu/franka-interface.git

COPY .docker_tmp/docker_util_scripts/build_robot_interface.sh /
RUN bash /build_robot_interface.sh
RUN rm /build_robot_interface.sh

COPY .docker_tmp/docker_util_scripts/build_catkin.sh /
RUN bash /build_catkin.sh
RUN rm /build_catkin.sh

COPY run_robot_interface_and_ros.sh /
CMD bash /run_robot_interface_and_ros.sh