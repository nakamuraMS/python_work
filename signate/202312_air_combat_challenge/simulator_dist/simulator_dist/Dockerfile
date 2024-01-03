FROM signate/runtime-gpu:dl_env

RUN apt-get -qq -y update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install \
        libboost-dev \
        libnlopt-cxx-dev \
        freeglut3-dev \
        lxde \
        tigervnc-standalone-server \
        tigervnc-common \
        novnc \
        websockify \
        git \
        zip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/local
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip && \
    unzip eigen-3.4.0.zip && \
    rm eigen-3.4.0.zip && \
    cd eigen-3.4.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install

WORKDIR /usr/local/include

COPY root/thirdParty/include/pybind11_json/pybind11_json.hpp pybind11_json/pybind11_json.hpp
ADD https://github.com/Neargye/magic_enum/releases/download/v0.9.3/magic_enum.hpp magic_enum/magic_enum.hpp
ADD https://raw.githubusercontent.com/bshoshany/thread-pool/v3.3.0/BS_thread_pool.hpp thread-pool/BS_thread_pool.hpp
RUN git clone https://github.com/nlohmann/json.git -b v3.11.2 --depth 1 && \
    cp -r json/include/nlohmann . && \
    rm -r json

RUN chmod 644 pybind11_json/pybind11_json.hpp magic_enum/magic_enum.hpp thread-pool/BS_thread_pool.hpp

RUN pip install --upgrade pip && \
    pip install jupyterlab==3.6.3 timeout-decorator==0.5.0 keras-rl==0.4.2 pfrl==0.3.0

WORKDIR /usr/local/src
COPY root ASRCAISim1
WORKDIR /usr/local/src/ASRCAISim1
RUN pip install .
WORKDIR /usr/local/src/ASRCAISim1/sample/modules/OriginalModelSample
RUN pip install .

WORKDIR /usr/local/src
RUN rm -rf ASRCAISim1 && pip cache purge && conda clean -a -y && rm -r /usr/local/eigen-3.4.0

RUN useradd -m signate
USER signate
WORKDIR /workspace