#From inside this folder
# docker build -t cogrob/ebt-dox .

FROM cogrob/ebt-gui
MAINTAINER Cognitive Robotics "http://cogrob.org/"

RUN apt-get install -y qtcreator 
RUN apt-get install -y firefox 

RUN wget -q -O - "https://dl-ssl.google.com/linux/linux_signing_key.pub" | sudo apt-key add - \
 && echo "deb http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google-chrome.list \
 && apt-get update \
 && apt-get install -y google-chrome-stable

#Add basic user
ENV USERNAME dox
ENV HOME /home/$USERNAME
ENV PULSE_SERVER /run/pulse/native
RUN mkdir -p $HOME
RUN useradd $USERNAME
RUN echo "$USERNAME:$USERNAME" | chpasswd
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN chown -R $USERNAME:$USERNAME $HOME

# Fix for qt and X server errors
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc 

# change user
USER dox
# WORKDIR /home/dox/
# ENTRYPOINT ["/home/dox/"]
