before_install() {
  echo Number of cores: $(nproc)
  echo Number of cores used: $(($(nproc)/2))
  cd /tmp
  git clone https://github.com/Itseez/opencv.git
  cd opencv/ \
  	&& ls \
	&& git checkout 2.4.9 \
	&& mkdir build \
	&& cd build \
	&& cmake -BUILD_opencv_nonfree="True" .. \
	&& make -j$(($(nproc)/2)) \
	&& sudo make -j$(nproc) install
}

APT_CORE='
cmake
libboost-dev
libglew-dev
freeglut3-dev
build-essential
libxmu-dev
libxi-dev
checkinstall
libgtk2.0-dev
pkg-config
'

sudo apt-get -qq --yes --force-yes install $APT_CORE

(before_install)
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/local.conf'
sudo ldconfig