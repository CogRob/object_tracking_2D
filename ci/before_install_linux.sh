before_install() {
  cd /tmp
  git clone https://github.com/Itseez/opencv.git

  cd opencv/ \
  	&& ls \
	&& git checkout 2.4.9 \
	&& mkdir build \
	&& cd build \
	&& cmake -BUILD_opencv_nonfree="True" .. \
	&& make -j$(nproc) \
	&& sudo make -j$(nproc) install
}

APT_CORE='
cmake
libboost-all-dev
libglew-dev
freeglut3-dev
liblapack3
build-essential
libxmu-dev
libxi-dev
checkinstall
libgtk2.0-dev
pkg-config
'

sudo apt-get -qq --yes --force-yes install $APT_CORE

(before_install)