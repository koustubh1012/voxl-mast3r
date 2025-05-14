FROM ubuntu:22.04 

LABEL description="Docker container for ROS2 Huble with dependencies for LightGlue installed"

# Set default shell
SHELL ["/bin/bash", "-c"]

ENV HOME=/root
WORKDIR ${HOME}
ENV DEBIAN_FRONTEND=noninteractive

# Set the locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

# Set device type
ENV DEVICE="cpu"

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# Set the timezone
ENV TZ=America/New_York
RUN apt-get update && apt-get install -y tzdata \
    && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone

# Base system dependencies and apps
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    lsb-release \
    terminator \
    pip

# ROS 2 Humble install -- FIXED
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    libopencv-dev \
    ros-humble-cv-bridge

# Clone voxl-master repo and px4_msgs
RUN mkdir -p ros2_ws/src \
    && cd ros2_ws/src \
    && git clone https://github.com/koustubh1012/voxl-mast3r.git \
    && git clone https://github.com/PX4/px4_msgs.git \
    && cd px4_msgs \
    && git checkout release/1.14

# Install Lightglue dependencies
RUN pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cpu \
    && pip install opencv-python \
    && pip install kornia \
    && pip install matplotlib

# Install LightGlue
RUN git clone https://github.com/cvg/LightGlue.git && cd LightGlue \
    && python3 -m pip install .

# Create torch cache directory
RUN mkdir -p /root/.cache/torch/hub/checkpoints

# Download LightGlue SIFT weights (v0.1_arxiv)
RUN curl -L -o /root/.cache/torch/hub/checkpoints/sift_lightglue_v0-1_arxiv.pth \
    https://github.com/cvg/LightGlue/releases/download/v0.1_arxiv/sift_lightglue.pth
