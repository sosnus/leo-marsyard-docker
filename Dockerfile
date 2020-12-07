

FROM ros:kinetic-ros-core-xenial


RUN apt-get update
RUN apt install python3-pip -y
RUN pip3 install web.py
RUN apt install tmux -y
RUN apt install htop -y

EXPOSE 1883	

CMD tmux new-session \; split-window -h \; split-window -v \; split-window -v "htop" \;
