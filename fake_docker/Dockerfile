#################################################
### IMPORT NVIDIA docker for Jetson Xavier NX ###
#################################################
FROM nvcr.io/nvidia/l4t-ml:r32.6.1-py3
#WORKDIR /usr/src/librealsense-docker


###########################################################
#### INSTALL Utils packages, libraries and dependencies ###
###########################################################
RUN apt-get update && apt-get install net-tools && yes | apt-get install curl
#RUN sudo apt-get install apt-utils
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*
RUN apt update


#####################################
### INSTALL PyRealSense libraries ###
#####################################
# https://cognitivexr.at/blog/2021/07/29/installing-pyrealsense2-nvidia-jetson-xavier-nx.html
RUN apt install -y --no-install-recommends \
    python3-setuptools \
    python3-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    unzip \
    sudo
RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.48.0.zip && \
    unzip v2.48.0.zip && \
    cd librealsense-2.48.0 && \
    mkdir build && cd build && \
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64 && \
    export PATH=$PATH:$CUDA_HOME/bin && \
    cmake ../ -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_WITH_CUDA=true && \
    make -j4 && \
    sudo make install
ENV PYTHONPATH "${PYTHONPATH}:/usr/local/lib/python3.6/pyrealsense2"