#From inside this folder
# docker build -t cogrob/ebt-nvidia .

FROM cogrob/ebt-dox
MAINTAINER Cognitive Robotics "http://cogrob.org/"

RUN sudo apt-get update
RUN sudo apt-get install -y x-window-system
RUN sudo apt-get install -y binutils
RUN sudo apt-get install -y mesa-utils
RUN sudo apt-get install -y module-init-tools

ADD nvidia-driver.run /tmp/nvidia-driver.run
RUN sudo sh /tmp/nvidia-driver.run -a -N --ui=none --no-kernel-module
RUN sudo rm /tmp/nvidia-driver.run

