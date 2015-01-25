#From inside this folder
# docker build -t cogrob/ebt-dev .
# docker run -t -i -v /path/to/local/omnimapper/repo:~/git/omnimapper  cogrob/ebt-dev /bin/bash
# docker export ebt-dev | gzip -c > ebt-dev.tgz
# docker import ebt-dev < ebt-dev.tgz

FROM cogrob/ebt-dep
MAINTAINER Cognitive Robotics "http://cogrob.org/"

RUN apt-get -y update

RUN apt-get -y install cmake cmake-curses-gui
RUN apt-get -y install libboost-all-dev

# libglide3 glew-utils libxcb-doc libxext-doc

RUN apt-get -y install libglew-dev
RUN apt-get -y install freeglut3-dev
RUN apt-get -y install liblapack3

RUN apt-get -y install build-essential libxmu-dev libxi-dev checkinstall
RUN apt-get -y install libgtk2.0-dev pkg-config

# RUN apt-get -y install libflann-dev
# RUN apt-get -y install libgsl0-dev
# RUN apt-get -y install libgoogle-perftools-dev

CMD /bin/bash
