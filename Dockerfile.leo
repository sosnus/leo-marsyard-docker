FROM osrf/ros:melodic-desktop-full

# Install some basic dependencies
RUN apt-get update && apt-get -y upgrade && \
  apt-get -y install curl ssh lsb-core g++ openssh-server gedit vim python-pip python3-pip && rm -rf /var/lib/apt/lists/*

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && sudo rosdep fix-permissions && rosdep update

RUN apt-get update && apt-get install -y gdb gnupg2 apt-transport-https

COPY ./rosbag/ /rosbag/
RUN ls /rosbag/

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Set root password
RUN echo 'root:root' | chpasswd

# Permit SSH root login
RUN sed -i 's/#*PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config
