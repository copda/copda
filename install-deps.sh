#!/bin/bash
set -e -x

# This script must be run in the in catkin_ws/src/ directory, i.e., the parent
# directory of this repo.

export ROS_DISTRO="noetic"

# clone dependencies
sudo apt-get update -qq
sudo apt-get install -qq -y python3-wstool git
if [ ! -f .rosinstall ]; then
  wstool init
fi
wstool merge --merge-keep -y copda/dependencies.rosinstall
wstool update

# install newest version of pyswip
# - the unreleased pyswip master version is compatible with swi-prolog 9.0.4 from the PPA
# - the latest released pyswip version (0.2.10) is compatible with swi-prolog 7.6.4 from the official Ubuntu repos
# - we currenty require the newest pyswip version
sudo apt-get install -qq -y software-properties-common
sudo apt-add-repository -y ppa:swi-prolog/stable
sudo apt install -y swi-prolog

pip install --upgrade git+https://github.com/yuce/pyswip@master#egg=pyswip

# use rosdep to install all dependencies (including ROS itself)
sudo apt-get install -qq -y python3-rosdep
sudo rosdep init > /dev/null 2>&1 || true
rosdep update
rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}
sudo apt-get install -qq -y python3-catkin-tools build-essential
sudo apt-get install -qq -y ccache

# install pip dependencies
pip install rdflib
