#From inside this folder
# docker build -t cogrob/ebt-gui .

FROM cogrob/ebt-dev
MAINTAINER Cognitive Robotics "http://cogrob.org/"

RUN apt-get update \
 && apt-get install -y xz-utils file locales dbus-x11 pulseaudio dmz-cursor-theme \
      fonts-dejavu fonts-liberation hicolor-icon-theme \
      libcanberra-gtk3-0 libcanberra-gtk-module libcanberra-gtk3-module \
      libasound2 libgtk2.0-0 libdbus-glib-1-2 libxt6 libexif12 \
      libgl1-mesa-glx libgl1-mesa-dri \
      wget byobu fish \
 && update-locale LANG=C.UTF-8 LC_MESSAGES=POSIX

RUN apt-get install -y terminator 
RUN apt-get install -y cmake-qt-gui 
RUN apt-get install -y gedit
