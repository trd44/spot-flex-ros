FROM ros:humble-ros-base-jammy

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Add the Gazebo package repository. Modern Gazebo packages are hosted by OSRF,
# not in the base ROS image repositories.
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Core build tools and ROS packages
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    curl \
    wget \
    git \
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-distutils \
    python3-apt \
    python3-numpy \
    python3-yaml \
    libpython3-dev \
    python3-tk \
    xauth \
    libegl1-mesa \
    libgl1-mesa-dri \
    libglu1-mesa \
    libglx-mesa0 \
    mesa-utils \
    x11-utils \
    x11-xserver-utils \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    qttools5-dev \
    ros-humble-ros-base \
    ros-dev-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-tl-expected \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    gz-fortress \
    ros-humble-ign-ros2-control \
    ros-humble-ros-gz \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

# Boston Dynamics SDK version (must match spot_ros2/spot_wrapper)
ARG SPOT_SDK_VERSION="5.0.1"

# Install bosdyn_msgs (BD protobuf -> ROS msg conversions)
RUN ARCH=$(dpkg --print-architecture) && \
    wget -q -O /tmp/bosdyn_msgs.run \
    "https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/${SPOT_SDK_VERSION}/ros-humble-bosdyn_msgs_${SPOT_SDK_VERSION}-jammy_${ARCH}.run" && \
    chmod +x /tmp/bosdyn_msgs.run && \
    bash -c 'yes | /tmp/bosdyn_msgs.run --nox11 || test $? -eq 141' && \
    rm /tmp/bosdyn_msgs.run

# Install spot-cpp-sdk
RUN ARCH=$(dpkg --print-architecture) && \
    wget -q -O /tmp/spot-cpp-sdk.deb \
    "https://github.com/bdaiinstitute/spot-cpp-sdk/releases/download/v${SPOT_SDK_VERSION}/spot-cpp-sdk_${SPOT_SDK_VERSION}_${ARCH}.deb" && \
    dpkg -i /tmp/spot-cpp-sdk.deb && \
    rm /tmp/spot-cpp-sdk.deb

# Install spot_wrapper python requirements (includes bosdyn-client, etc.)
COPY workspace/src/spot_ros2/spot_wrapper/requirements.txt /tmp/spot_wrapper_requirements.txt
RUN pip install --no-cache-dir -r /tmp/spot_wrapper_requirements.txt && \
    rm /tmp/spot_wrapper_requirements.txt

# Perception ML dependencies and model cache.
ENV SPOT_FLEX_MODEL_DIR=/opt/spot_flex_model_cache
ENV HF_HOME=/opt/spot_flex_model_cache/huggingface
ENV TRANSFORMERS_CACHE=/opt/spot_flex_model_cache/huggingface/hub

RUN pip install --no-cache-dir \
    "Pillow>=10.4,<12" \
    torch \
    torchvision \
    transformers \
    segment-anything && \
    mkdir -p "${SPOT_FLEX_MODEL_DIR}/sam" "${HF_HOME}" && \
    python3 -c "from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor; model_id='google/owlv2-base-patch16-ensemble'; AutoProcessor.from_pretrained(model_id); AutoModelForZeroShotObjectDetection.from_pretrained(model_id)" && \
    wget -q -O "${SPOT_FLEX_MODEL_DIR}/sam/sam_vit_b_01ec64.pth" \
      "https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth"

# Pin setuptools version that spot_ros2 expects
RUN pip install --no-cache-dir --force-reinstall "setuptools==59.6.0"

# Initialize rosdep
RUN rosdep init || true && rosdep update

WORKDIR /repo

ENV RCUTILS_COLORIZED_OUTPUT=1
