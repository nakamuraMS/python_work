#!/bin/bash

SIMDIR=./root

# boost, nlopt, vnc, novnc
apt-get -qq -y update && DEBIAN_FRONTEND=noninteractive apt-get -y install libboost-dev libnlopt-cxx-dev freeglut3-dev wget unzip git \
        gcc \
        cmake \
        lxde \
        xvfb \
        tigervnc-standalone-server \
        tigervnc-common \
        novnc \
        websockify \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# pybind
mkdir /usr/local/include/pybind11_json && cp ${SIMDIR}/thirdParty/include/pybind11_json/pybind11_json.hpp /usr/local/include/pybind11_json/pybind11_json.hpp

# magic_enum
wget https://github.com/Neargye/magic_enum/releases/download/v0.9.3/magic_enum.hpp -P /usr/local/include/magic_enum/

# thread-pool
wget https://raw.githubusercontent.com/bshoshany/thread-pool/v3.3.0/BS_thread_pool.hpp -P /usr/local/include/thread-pool/

# nlohmann
git clone https://github.com/nlohmann/json.git -b v3.11.2 --depth 1 && \
    cp -r json/include/nlohmann /usr/local/include

chmod 644 /usr/local/include/pybind11_json/pybind11_json.hpp usr/local/include/magic_enum/magic_enum.hpp /usr/local/include/thread-pool/BS_thread_pool.hpp

# eigen
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip && \
    unzip eigen-3.4.0.zip && \
    rm eigen-3.4.0.zip && \
    cd eigen-3.4.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install

cd ../..
rm -r eigen-3.4.0
rm -r json