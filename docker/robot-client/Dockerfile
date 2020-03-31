FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu16.04

RUN apt-get update
RUN apt-get install -y curl git wget iputils-ping

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y software-properties-common python-software-properties

# === Install Python 3.6 ===

RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update
RUN apt-get -y install python3.6 python3.6-dev

# Pip 3.6
RUN wget https://bootstrap.pypa.io/get-pip.py
RUN python3.6 get-pip.py
RUN rm get-pip.py

# Python system-wide packages
RUN pip3.6 install numpy matplotlib scipy scikit-learn pandas seaborn ipython jupyter scikit-image opencv-python virtualenv
RUN pip3.6 install torch==1.2.0 torchvision==0.4.0
RUN pip3.6 install tensorflow-gpu

# Make Python virtualenv
RUN mkdir ~/venvs
RUN virtualenv --system-site-packages ~/venvs/robot-client
RUN echo "source ~/venvs/robot-client/bin/activate" >> ~/.bashrc

# === Install franka-interface related packages === 

RUN apt-get install -y cmake build-essential
RUN apt-get install -y libeigen3-dev libboost-all-dev 
RUN apt-get install -y libpoco-doc libpoco-dev

COPY .docker_tmp/docker_util_scripts/install_ros.sh /
RUN bash /install_ros.sh
RUN rm /install_ros.sh

ARG GITHUB_TOKEN
RUN git clone --recurse-submodules https://$GITHUB_TOKEN:@github.com/iamlab-cmu/franka-interface.git

COPY .docker_tmp/docker_util_scripts/build_robot_interface.sh /
RUN bash /build_robot_interface.sh
RUN rm /build_robot_interface.sh

COPY .docker_tmp/docker_util_scripts/build_catkin.sh /
RUN bash /build_catkin.sh
RUN rm /build_catkin.sh

# Custom venv-specific packages
COPY install_custom_python_pkgs.sh /
RUN bash /install_custom_python_pkgs.sh
RUN rm /install_custom_python_pkgs.sh

ARG ROBOT_SERVER_IP
ENV ROBOT_SERVER_IP ${ROBOT_SERVER_IP}

RUN echo "source /franka-interface/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://${ROBOT_SERVER_IP}:11311" >> ~/.bashrc

CMD ["/bin/bash", "-lc", \
    "source ~/venvs/robot-client/bin/activate && \
     export ROS_MASTER_URI=http://$ROBOT_SERVER_IP:11311 && \
     source /franka-interface/catkin_ws/devel/setup.bash && \
     cd /robot_client_io && chmod +x run.sh && ./run.sh"]
