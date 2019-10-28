#!/bin/bash

project_path=$(cd `dirname $0`; pwd)
set -e
# Homebrew
if ! hash brew 2>/dev/null; then
    echo "Installing brew"
    /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
fi
echo "brew tap homebrew/cask"
brew tap homebrew/cask
echo "brew tap osrf/simulation"
brew tap osrf/simulation
if [ ! -d $(brew --repo)/Library/Taps/ros/homebrew-deps ]; then
    echo "brew tap ros/deps"
    mkdir -p $(brew --repo)/Library/Taps/ros
    cd $(brew --repo)/Library/Taps/ros
    git clone https://github.com/nagakiran/homebrew-deps.git
    brew tap ros/deps
fi
brew update

brew install python@2
mkdir -p ~/Library/Python/2.7/lib/python/site-packages
if [ ! -f ~/Library/Python/2.7/lib/python/site-packages/homebrew.pth ]; then
    echo "$(brew --prefix)/lib/python2.7/site-packages" >>~/Library/Python/2.7/lib/python/site-packages/homebrew.pth
fi

brew install python
mkdir -p ~/Library/Python/3.7/lib/python/site-packages
if [ ! -f ~/Library/Python/3.7/lib/python/site-packages/homebrew.pth ]; then
    echo "$(brew --prefix)/lib/python3.7/site-packages" >>~/Library/Python/3.7/lib/python/site-packages/homebrew.pth
fi

brew cask install xquartz
brew install gpgme
brew install poco
brew install gtest
brew install lz4
brew install fltk
brew install boost-python3
brew install yaml-cpp
brew install opencv
brew install pcl
brew install log4cxx

sudo -H python2 -m pip install -U pip
sudo -H python3 -m pip install -U pip
sudo -H python2 -m pip install -U wstool rosdep rosinstall rosinstall_generator rospkg catkin-pkg sphinx
sudo -H python3 -m pip install -U wstool rosdep rosinstall rosinstall_generator rospkg catkin-pkg sphinx gnupg pydot

cd $project_path
if [ ! -d /etc/ros/rosdep/ ]; then
    sudo rosdep init
fi
if [ ! -f /etc/ros/rosdep/sources.list.d/10-ros-install-osx.list ]; then
    echo "This sudo prompt adds the the brewed python rosdep yaml to /etc/ros/rosdep/sources.list.d/10-ros-install-osx.list"
    sudo sh -c "echo 'yaml file://$(pwd)/rosdeps.yaml osx' > /etc/ros/rosdep/sources.list.d/10-ros-install-osx.list"
fi
rosdep update
rosinstall_generator desktop_full --rosdistro melodic --deps --wet-only --tar >melodic-desktop-full-wet.rosinstall

wstool init src
wstool merge file://$(pwd)/melodic-desktop-full-wet.rosinstall
wstool update
# wstool init -j8 src melodic-desktop-full-wet.rosinstall
# wstool update -j 8 -t src
rosdep install --from-paths src --ignore-src --rosdistro melodic -y

# patch
get_filelist() {
    flist=()
    index=1
    for file in $(ls $1); do
        if [ -d $1$file ]; then
            get_filelist $1$file"//"
        else
            echo $1$file
        fi
    done
}
cd ~/melodic_catkin_ws/patch/
result=($(get_filelist ./))
for f in $result[*]; do
    echo "patch/"$f"to""src/"$f
    cp $(pwd)"/patch/"$f $(pwd)"/src"$f
done

export PATH=/usr/local/opt/qt/bin:$PATH
export LIBRARY_PATH=/usr/local/lib:$LIBRARY_PATH
export CPATH=/usr/local/include/harfbuzz:$CPATH
export PKG_CONFIG_PATH="/usr/local/opt/openssl@1.1/lib/pkgconfig"

sudo rm -rf /opt/ros/melodic
sudo mkdir -p /opt/ros/melodic
sudo chown $USER /opt/ros/melodic
./src/catkin/bin/catkin_make_isolated --install --install-space=/opt/ros/melodic \
    --cmake-args -DCATKIN_ENABLE_TESTING=1 -DCMAKE_CXX_STANDARD=11 -DBoost_NO_BOOST_CMAKE=ON \
    -DCMAKE_MACOSX_RPATH=ON -DCMAKE_INSTALL_RPATH="/opt/ros/melodic/lib" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_FIND_FRAMEWORK=LAST \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DPYTHON_LIBRARY=$(python3 -c "import sys; print(sys.prefix)")/lib/libpython3.7.dylib \
    -DPYTHON_INCLUDE_DIR=$(python3 -c "import sys; print(sys.prefix)")/include/python3.7m -DGTEST_SRC_DIR="/usr/local/opt/gtest"
