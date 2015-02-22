#From inside this folder
# docker build -t cogrob/ebt-demo .

############################################################
# Dockerfile to build EBT images
# Based on Ubuntu
############################################################

FROM cogrob/ebt-dox
MAINTAINER Cognitive Robotics "http://cogrob.org/"

USER dox

# Download OpenCV
WORKDIR /home/dox
RUN git clone https://github.com/Itseez/opencv.git 

# Build and Install OpenCV
RUN cd /home/dox/opencv/ \
	&& git checkout 2.4.9 \
	&& mkdir build \
	&& cd build \
	&& cmake -BUILD_opencv_nonfree="True" .. \
	&& make -j8 \
	&& sudo make install

# Download EBT
WORKDIR /home/dox
RUN git clone https://github.com/CognitiveRobotics/object_tracking_2D.git

# Build EBT
RUN cd /home/dox/object_tracking_2D/ \
	&& mkdir build \
	&& cd build \
	&& cmake .. \
	&& make -j8

# WORKDIR /home/dox
# RUN sudo chown dox /home/dox/.bashrc