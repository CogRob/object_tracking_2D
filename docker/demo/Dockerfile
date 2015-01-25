#From inside this folder
# docker build -t cogrob/omnimapper-demo .

FROM cogrob/omnimapper-dox
MAINTAINER Cognitive Robotics "http://cogrob.org/"

USER dox
WORKDIR /home/dox
RUN wget https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam-3.2.0.tgz \
	&& mkdir gtsam \
	&& tar -xvzf gtsam-3.2.0.tgz --strip=1 -C gtsam/ \
	&& rm gtsam-3.2.0.tgz

RUN cd /home/dox/gtsam/ \
	&& mkdir build \
	&& cd build \
	&& cmake .. \
	&& make -j8 \
	&& sudo make install

RUN sudo apt-get install -y libeigen3-dev
RUN sudo apt-get install -y ros-indigo-perception-pcl
RUN sudo apt-get install -y ros-indigo-openni-launch

COPY HouseholderQR.h /usr/include/eigen3/Eigen/src/QR/HouseholderQR.h
COPY CommaInitializer.h /usr/include/eigen3/Eigen/src/CommaInitializer.h
RUN sudo chmod 644 /usr/include/eigen3/Eigen/src/QR/HouseholderQR.h
RUN sudo chmod 644 /usr/include/eigen3/Eigen/src/CommaInitializer.h

WORKDIR /home/dox
RUN git clone https://github.com/CognitiveRobotics/omnimapper.git

RUN cd /home/dox/omnimapper/ \
	&& mkdir build \
	&& cd build \
	&& cmake .. \
	&& make -j8

WORKDIR /home/dox
RUN sudo chown dox /home/dox/.bashrc

ADD build.sh /home/dox/build.sh
RUN sudo chown dox /home/dox/build.sh
RUN bash /home/dox/build.sh
