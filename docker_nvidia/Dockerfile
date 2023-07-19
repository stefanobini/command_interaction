FROM docker.io/nvidia/cuda:11.4.3-devel-ubuntu18.04
#FROM docker.io/andrewseidl/nvidia-cuda:10.2-devel-ubuntu20.04
ARG DEBIAN_FRONTEND=noninteractive

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
RUN apt update
RUN apt-get install python3-rospkg -y
RUN yes | apt-get install python-rosdep python-rosinstall-generator python-vcstool python-rosinstall build-essential
RUN apt install python3-pip -y
RUN apt install ros-melodic-ros-base -y
RUN apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
#RUN apt install python3-pip -y
RUN python3 -m pip install -U catkin_tools
RUN apt-get install ros-melodic-rospy -y

RUN pip3 install rospkg 

COPY ./ros_entrypoint.sh /
#COPY ./qi_install.sh /

#RUN wget https://github.com/robertanto/libqi-python-nvidia-jetson/releases/download/1.0/qi_jetson.tar.gz && \
#    tar -xvzf qi_jetson.tar.gz && \
#    cd qi_jetson && \
#    chmod u+x install.sh && \
#    ./install.sh

# RUN ["chmod", "+x", "/ros_entrypoint.sh"]

ENTRYPOINT ["/ros_entrypoint.sh"]

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
#RUN python3 -m pip install torchmetrics==0.6.0
#RUN python3 -m pip install torchmetrics==0.6.0
#RUN python3 -m pip install torch==1.9.0 torchvision==0.10.0+cu102 torchaudio==0.9.0 -f https://torch.kmtea.eu/whl/stable.html

#RUN python3 -m pip install git+https://github.com/NVIDIA/NeMo.git@v1.4.0#egg=nemo_toolkit[asr]
RUN python3 -m pip install setuptools==59.5.0

#Utils installation
RUN python3 -m pip install SpeechRecognition
RUN apt install libsndfile1 -y
RUN python3 -m pip install webrtcvad
RUN python3 -m pip install netifaces
#RUN python3 -m pip install -U jetson-stats

RUN apt install python-pip -y
RUN python3 -m pip install colorama
RUN python -m pip install colorama

#Utils DemoFramework
RUN pip3 install cvbridge3
RUN sudo pip3 install flask
RUN sudo apt install python-cv-bridge -y
RUN sudo apt install python-opencv -y

# DOTMAP
RUN pip2 install dotmap==1.3.23 

# RUN apt-get install -q -y net-tools 
RUN python -m pip install ifcfg

# Python packages
RUN python3 -m pip install soundfile librosa dotmap
RUN python3 -m pip install pandas
#RUN python3 -m pip install torch==1.12.1+cu102 torchvision==0.13.1+cu102 torchaudio==0.12.1 --extra-index-url https://download.pytorch.org/whl/cu102
RUN python3 -m pip install torch torchaudio torchvision torchmetrics --extra-index-url https://download.pytorch.org/whl/cu113
RUN python3 -m pip install numpy pytorch-lightning torchmetrics torchsummary tensorboard

CMD ["bash"]