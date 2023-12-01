FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y -q \
    git \
    build-essential \
    cmake \
    python3 \
    qtbase5-dev \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev \
    libtinfo-dev \
    libncurses5-dev \
    libncursesw5-dev

RUN git clone https://github.com/BehaviorTree/Groot.git &&\
    cd Groot &&\
    git submodule update --init --recursive &&\
    cd depend/BehaviorTree.CPP &&\
    mkdir build ; cd build &&\
    cmake .. &&\
    make &&\
    make install &&\
    cd /Groot &&\
    mkdir build; cd build &&\
    cmake .. &&\
    make &&\
    # add alias for Groot GUI (simply type "Groot" in terminal)
    echo 'alias Groot="cd /Groot/build && ./Groot"' >> ~/.bashrc

# fixes: "error while loading shared libraries: libbehaviortree_cpp_v3.so: cannot open shared object"
RUN ldconfig

WORKDIR /Groot/build
CMD ./Groot
