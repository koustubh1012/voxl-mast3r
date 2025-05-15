FROM ubuntu:22.04 

LABEL description="Docker container for ROS2 Humble with dependencies for LightGlue installed"

# Set default shell
SHELL ["/bin/bash", "-c"]

# -----------------------
# 1. Create non-root user
# -----------------------
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} user && \
    useradd -m -u ${UID} -g ${GID} user

ENV HOME=/home/user
WORKDIR ${HOME}
ENV DEBIAN_FRONTEND=noninteractive

# -----------------------
# 2. Set the locale
# -----------------------
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# -----------------------
# 3. Set the timezone
# -----------------------
ENV TZ=America/New_York
RUN apt-get update && apt-get install -y tzdata \
    && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone

# -----------------------
# 4. Base system packages
# -----------------------
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    lsb-release \
    terminator \
    pip \
    sudo

# -----------------------
# 5. ROS 2 Humble install
# -----------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    libopencv-dev \
    ros-humble-cv-bridge

# -----------------------
# 6. Clone and build workspace
# -----------------------
USER user
WORKDIR /home/user
RUN mkdir -p ros2_ws/src

# Clone repos
RUN cd ros2_ws/src && \
    git clone https://github.com/koustubh1012/voxl-mast3r.git && \
    git clone https://github.com/PX4/px4_msgs.git && \
    cd px4_msgs && \
    git checkout release/1.14

# Source ROS and build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /home/user/ros2_ws && \
    colcon build"

# Add sourcing to .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/user/.bashrc && \
    echo "source /home/user/ros2_ws/install/setup.bash" >> /home/user/.bashrc

# -----------------------
# 7. Optional: LightGlue install
# -----------------------
# RUN pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cpu \
#     && pip install opencv-python \
#     && pip install kornia \
#     && pip install matplotlib
# RUN git clone https://github.com/cvg/LightGlue.git && cd LightGlue \
#     && python3 -m pip install .
# RUN mkdir -p /home/user/.cache/torch/hub/checkpoints
# RUN curl -L -o /home/user/.cache/torch/hub/checkpoints/sift_lightglue_v0-1_arxiv.pth \
#     https://github.com/cvg/LightGlue/releases/download/v0.1_arxiv/sift_lightglue.pth

# -----------------------
# 8. EntryPoint
# -----------------------
# COPY entrypoint.sh /usr/local/bin/entrypoint.sh
# RUN chmod +x /usr/local/bin/entrypoint.sh

COPY --chmod=755 entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
