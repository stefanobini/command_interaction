FROM nvcr.io/nvidia/l4t-ml:r32.6.1-py3
#WORKDIR /usr/src/librealsense-docker

# install packages

RUN apt-get update && apt-get install -y apt-transport-https
RUN apt-get install net-tools
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic

# install ros packages

#RUN apt-get update && apt-get install -y --no-install-recommends \
#    ros-melodic-ros-core=1.4.1-0* \
#    && rm -rf /var/lib/apt/lists/*

RUN apt update
RUN apt-get install python3-rospkg -y
RUN apt install ros-melodic-ros-base -y
RUN apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
RUN apt install python3-pip -y
RUN python3 -m pip install -U catkin_tools
RUN apt-get install ros-melodic-rospy -y

RUN sudo pip3 install rospkg && \
    sudo apt install -y ros-melodic-naoqi-bridge-msgs && \
    sudo apt install -y ros-melodic-vision-msgs

COPY ./ros_entrypoint.sh /
COPY ./qi_install.sh /

RUN wget https://github.com/robertanto/libqi-python-nvidia-jetson/releases/download/1.0/qi_jetson.tar.gz && \
    tar -xvzf qi_jetson.tar.gz && \
    cd qi_jetson && \
    chmod u+x install.sh && \
    ./install.sh

# RUN ["chmod", "+x", "/ros_entrypoint.sh"]

ENTRYPOINT ["/ros_entrypoint.sh"]

# Pyrealsense 2 installation
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-setuptools \
        python3-dev \
        libssl-dev \
        libusb-1.0-0-dev \
        pkg-config \
        libgtk-3-dev \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        sudo 

#RUN git clone https://github.com/IntelRealSense/librealsense.git &&\
    #cd librealsense && \
    #./scripts/setup_udev_rules.sh && \
    #mkdir build && \
    #cd build && \
    #cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true && \
    #sudo make uninstall && \
    #sudo make clean && \
    #sudo make -j4 && \
    #sudo make install 

ENV PYTHONPATH "${PYTHONPATH}:/usr/local/lib/python3.6/pyrealsense2"

# ReSpeaker installation
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        portaudio19-dev \
        python-pyaudio \
        flac 

RUN pip3 install \
        PyAudio==0.2.11 \
        pyusb==1.0.2 \
        click==7.1.2

RUN python3 -m pip install --upgrade pip
RUN apt-get install python3-setuptools -y
RUN apt install curl -y	
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

#RUN apt-get install libportaudio2 -y
#RUN python3 -m pip install torch==1.9.0 torchaudio torchmetrics==0.6.0
RUN python3 -m pip install torchmetrics==0.6.0
#RUN python3 -m pip install torchmetrics==0.6.0
#RUN python3 -m pip install torch==1.9.0 torchvision==0.10.0+cu102 torchaudio==0.9.0 -f https://torch.kmtea.eu/whl/stable.html

RUN python3 -m pip install git+https://github.com/NVIDIA/NeMo.git@v1.4.0#egg=nemo_toolkit[asr]
RUN python3 -m pip install setuptools==59.5.0

#Utils installation
RUN python3 -m pip install SpeechRecognition
RUN python3 -m pip install dotmap
RUN apt install libsndfile1 -y
RUN python3 -m pip install sounddevice
RUN python3 -m pip install webrtcvad
RUN python3 -m pip install netifaces
RUN python3 -m pip install -U jetson-stats

RUN apt install python-pip -y
RUN python3 -m pip install colorama
RUN python -m pip install colorama

#Utils DemoFramework
RUN pip3 install cvbridge3
RUN sudo pip3 install flask
RUN sudo apt install python-cv-bridge -y
RUN sudo apt install python-opencv -y

#INSTALL PYREALSE
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
        sudo 

RUN git clone https://github.com/IntelRealSense/librealsense.git &&\
    cd librealsense && \
    ./scripts/setup_udev_rules.sh && \
    mkdir build && \
    cd build && \
    cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true && \
    sudo make uninstall && \
    sudo make clean && \
    sudo make -j4 && \
    sudo make install 

ENV PYTHONPATH "${PYTHONPATH}:/usr/local/lib/python3.6/pyrealsense2"

# DOTMAP
RUN pip2 install dotmap==1.3.23 

# RUN apt-get install -q -y net-tools 
RUN python -m pip install ifcfg

#RUN pip3 install pyrealsense2



#RUN git clone https://github.com/peppo97/tesi_magistrale_ros /home/

#GESTURE RECOGNITION (Marchesano)

#RUN python3 -m pip install opencv-python==4.5.5.64
RUN python3 -m pip install Pillow
#RUN python3 -m pip install torchvision==0.10.0

# ONNX libraries
pip3 install -U openmim
mim install mmcv-full
git clone https://github.com/open-mmlab/mmdetection.git
cd mmdetection
pip3 install -v -e .
# wget https://nvidia.box.com/shared/static/jy7nqva7l88mq9i8bw3g3sklzf4kccn2.whl -O onnxruntime_gpu-1.10.0-cp36-cp36m-linux_aarch64.whl
# pip3 install onnxruntime_gpu-1.10.0-cp36-cp36m-linux_aarch64.whl
# python3 -m pip install onnxruntime

# mim install mmdet==2.14.0
# git clone https://github.com/grimoire/torch2trt_dynamic.git torch2trt_dynamic
# cd torch2trt_dynamic
# python3 setup.py develop

#git clone --depth=1 https://github.com/grimoire/amirstan_plugin.git
#cd amirstan_plugin
#git submodule update --init --progress --depth=1
#mkdir build
#cd build
#cmake -DTENSORRT_DIR=/usr/src/tensorrt ..
#make -j10

CMD ["bash"]
