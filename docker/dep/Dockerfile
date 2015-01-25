# docker build -t cogrob/ebt-dep .
# docker export ebt-dep | gzip -c > ebt-dep.tgz
# docker import ebt-dep < ebt-dep.tgz

FROM ubuntu:14.04
MAINTAINER Cognitive Robotics "http://cogrob.org/"

# Intall some basic tools
RUN apt-get -y update
RUN apt-get -y install curl screen byobu fish git nano glances

CMD /bin/bash