#!/usr/bin/env bash

set -e

SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
cd "$SCRIPT_PATH/.."

# Install custom dependencies using vcstool

mkdir -p src/lib
vcs import src/lib --force --shallow < custom_deps.yaml

# Install or-tools and Fields2Cover
# todo handle eventually the case where or-tools or Fields2Cover is already installed

sudo apt-get update
sudo apt-get install -y --no-install-recommends software-properties-common
sudo add-apt-repository ppa:ubuntugis/ppa
sudo apt-get update
sudo apt-get install -y --no-install-recommends build-essential ca-certificates cmake \
     doxygen g++ git libeigen3-dev libgdal-dev libpython3-dev python3 python3-pip \
     python3-matplotlib python3-tk lcov libgtest-dev libtbb-dev swig libgeos-dev \
     gnuplot libtinyxml2-dev nlohmann-json3-dev
python3 -m pip install gcovr

git clone https://github.com/google/or-tools /tmp/or-tools
cd /tmp/or-tools
cmake -S . -B build -DBUILD_DEPS=ON
cmake --build build --config Release --target all -j -v
cmake --build build --config Release --target install -v

git clone https://github.com/Fields2Cover/Fields2Cover.git /tmp/fields2cover
cd /tmp/fields2cover
wget https://github.com/google/or-tools/releases/download/v9.9/or-tools_amd64_ubuntu-22.04_cpp_v9.9.3963.tar.gz
tar -xvf or-tools_amd64_ubuntu-22.04_cpp_v9.9.3963.tar.gz
sudo cp -r or-tools_x86_64_Ubuntu-22.04_cpp_v9.9.3963/bin/* /usr/local/bin/
sudo cp -r or-tools_x86_64_Ubuntu-22.04_cpp_v9.9.3963/include/* /usr/local/include/
sudo cp -r or-tools_x86_64_Ubuntu-22.04_cpp_v9.9.3963/lib/* /usr/local/lib/
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make instsall
