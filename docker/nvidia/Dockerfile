#From inside this folder
# docker build -t cogrob/ebt-nvidia .

############################################################
# Dockerfile to build EBT images
# Based on Ubuntu
############################################################

FROM cogrob/ebt-demo
MAINTAINER Cognitive Robotics "http://cogrob.org/"

RUN sudo apt-get update && sudo apt-get install -y \
	x-window-system \
	binutils \
	mesa-utils \
	module-init-tools

# ADD nvidia-driver.run /tmp/nvidia-driver.run
# RUN sudo sh /tmp/nvidia-driver.run -a -N --ui=none --no-kernel-module
# RUN sudo rm /tmp/nvidia-driver.run

ADD nvidia-driver.deb /tmp/nvidia-driver.deb
RUN sudo dpkg -i /tmp/nvidia-driver.deb \
	&& sudo apt-get update \
	&& sudo apt-get install -y --force-yes cuda \
	&& sudo rm /tmp/nvidia-driver.deb