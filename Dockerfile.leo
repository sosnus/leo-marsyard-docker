FROM osrf/ros:melodic-desktop-full

# Install some basic dependencies
RUN apt-get update && apt-get -y upgrade && \
  apt-get -y install curl ssh lsb-core g++ openssh-server gedit vim python-pip python3-pip && rm -rf /var/lib/apt/lists/*

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && sudo rosdep fix-permissions && rosdep update

RUN apt-get update && apt-get install -y gdb gnupg2 apt-transport-https

RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src && git clone https://github.com/PUT-UGV-Team/leo-marsyard.git && mv leo-marsyard/* . && rm -rf leo_marsyard/

RUN mkdir -p /root/.gazebo/models/terrain
RUN cp -r /catkin_ws/src/leo_gazebo/models/terrain/* /root/.gazebo/models/terrain/

RUN rosdep install --from-paths /catkin_ws/src/ --ignore-src --rosdistro melodic -r -y
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin_make'

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc


# Set root password
RUN echo 'root:root' | chpasswd

# Permit SSH root login
RUN sed -i 's/#*PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config

# Install catkin-tools
RUN apt-get update && apt-get install -y python-catkin-tools \
 ros-melodic-cv-bridge \
 ros-melodic-tf \
 ros-melodic-robot-localization \
 ros-melodic-interactive-marker-twist-server \
 ros-melodic-twist-mux \
 ros-melodic-pointcloud-to-laserscan \
 ros-melodic-dwa-local-planner \
 ros-melodic-moveit \
 ros-melodic-rviz-imu-plugin\
 ros-melodic-base-local-planner ros-melodic-clear-costmap-recovery \
 ros-melodic-compressed-depth-image-transport ros-melodic-compressed-image-transport \
 ros-melodic-costmap-2d ros-melodic-costmap-converter \
 ros-melodic-diff-drive-controller ros-melodic-dynamic-reconfigure* \
 ros-melodic-global-planner ros-melodic-navfn ros-melodic-navigation \
 ros-melodic-rosbag ros-melodic-rostopic ros-melodic-rqt* \
 ros-melodic-urdf ros-melodic-voxel-grid \
 ros-melodic-tf ros-melodic-octomap*\
 ros-melodic-teb-local-planner* ros-melodic-rtabmap-ros \
  && rm -rf /var/lib/apt/lists/*
