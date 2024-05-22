#################################################
### IMPORT NVIDIA docker for Jetson Xavier NX ###
#################################################
#FROM nvcr.io/nvidia/l4t-ml:r35.2.1-py3
#FROM nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3
FROM dustynv/l4t-ml:r35.2.1
#WORKDIR /usr/src/librealsense-docker


###########################################################
#### INSTALL Utils packages, libraries and dependencies ###
###########################################################
RUN apt-get update && apt-get install net-tools && yes | apt-get install curl
RUN yes | apt-get install apt-utils
RUN apt-get update && apt-get install -y lsb-release && apt-get clean all
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*
RUN apt update


#####################################
### INSTALL PyRealSense libraries ###
#####################################
## https://cognitivexr.at/blog/2021/07/29/installing-pyrealsense2-nvidia-jetson-xavier-nx.html
#RUN apt install -y --no-install-recommends \
#    python3-setuptools \
#    python3-dev \
#    libssl-dev \
#    libusb-1.0-0-dev \
#    pkg-config \
#    libgtk-3-dev \
#    libglfw3-dev \
#    libgl1-mesa-dev \
#    libglu1-mesa-dev \
#    unzip \
#    sudo
#RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.48.0.zip && \
#    unzip v2.48.0.zip && \
#    cd librealsense-2.48.0 && \
#    mkdir build && cd build && \
#    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64 && \
#    export PATH=$PATH:$CUDA_HOME/bin && \
#    cmake ../ -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_WITH_CUDA=true && \
#    make -j4 && \
#    sudo make install
#ENV PYTHONPATH "${PYTHONPATH}:/usr/local/lib/python3.8/pyrealsense2"


#############################################
### SETUP sources.list, keys, environment ###
#############################################
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
##RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 0F164EEB
##RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
#RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

#ENV LANG C.UTF-8
#ENV LC_ALL C.UTF-8
#ENV ROS_DISTRO noetic
## install ros packages
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    ros-noetic-ros-core=1.4.1-0* \
#    && rm -rf /var/lib/apt/lists/*


#########################################################
### INSTALL ROS Dependencies, Packages, and Libraries ###
#########################################################
RUN apt update
#RUN yes | apt-get install python3-rospkg -y
RUN yes | apt install ros-noetic-ros-base
RUN yes | apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN yes | apt install python3-catkin-tools
RUN apt install python3-rosdep
RUN apt install -y ros-noetic-vision-msgs
RUN rosdep init
RUN rosdep update


#################################
### INSTALL Python3 libraries ###
#################################
#RUN apt install python3-pip -y
RUN python3 -m pip install --upgrade pip
#ReSpeaker libraries
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        portaudio19-dev \
        python-all-dev \
        python3-all-dev \
        flac
RUN python3 -m pip install \
        PyAudio==0.2.11 \
        pyusb==1.0.2 \
        click==7.1.2
#RUN python3 -m pip install torchmetrics
#RUN python3 -m pip install setuptools
RUN python3 -m pip install dotmap
RUN python3 -m pip install colorama
RUN python3 -m pip install netifaces
RUN python3 -m pip install cvbridge3
RUN python3 -m pip install Pillow
RUN python3 -m pip install SpeechRecognition
RUN python3 -m pip install sounddevice
RUN python3 -m pip install webrtcvad
RUN python3 -m pip install soundfile
RUN python3 -m pip install librosa
RUN python3 -m pip install pytorch-lightning PyYAML==5.3.1
RUN python3 -m pip install flask blinker==1.4
RUN apt install python3-opencv -y


#####################
### INSTALL Utils ###
#####################
RUN apt-get update
RUN curl https://sh.rustup.rs -sSf | sh -s -- --default-toolchain nightly -y
ENV PATH="/root/.cargo/bin:${PATH}"
RUN apt install libsndfile1 -y

CMD ["bash"]
