FROM osrf/ros:melodic-desktop-full


RUN apt-get update && apt-get upgrade -y && \
	apt-get install -y lsb-core g++ openssh-server gedit vim

RUN apt-get install -y ros-melodic-robot-localization ros-melodic-interactive-marker-twist-server ros-melodic-twist-mux ros-melodic-pointcloud-to-laserscan ros-melodic-dwa-local-planner ros-melodic-moveit ros-melodic-rviz-imu-plugin

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

RUN apt-get update && apt-get install -y gdb gnupg2 apt-transport-https

RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src && git clone https://github.com/PUT-UGV-Team/leo-marsyard.git && mv leo-marsyard/leo_ws/src/* /catkin_ws/src/ && rm -rf leo-marsyard

RUN mkdir -p /root/.gazebo/models/terrain
RUN cp -r /catkin_ws/src/leo/leo_gazebo/models/terrain/* /root/.gazebo/models/terrain/

RUN rosdep install --from-paths /catkin_ws/src/ --ignore-src --rosdistro melodic -r -y
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin_make'

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
