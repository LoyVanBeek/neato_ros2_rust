ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base
LABEL authors="loy"

RUN apt update -qq && apt install -y --no-install-recommends \
      apt-transport-https \
      apt-utils \
      bash-completion \
      ca-certificates \
      dpkg \
      gdb \
      git \
      git-lfs \
      software-properties-common \
      tree \
      vim \
      wget \
      iproute2 \
      net-tools \
      iputils-ping \
      netcat-traditional \
    && rm -rf /var/lib/apt/lists/*

# install_ros_tools
RUN apt update -qq && apt install -y --no-install-recommends \
      curl \
      gnupg2 \
      lsb-release \
      python3-pip \
      ros-dev-tools \
      python3-argcomplete \
      tmux \
    && rm -rf /var/lib/apt/lists/*

# Install Rust (see https://rustup.rs/)
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs >> /rustup.sh
RUN sh /rustup.sh -y

# Install dependencies for rclrs and colcon-cargo & colcon-ros-cargo
RUN apt update -qq && apt install -y --no-install-recommends \
      git \
    libclang-dev \
    python3-pip \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install colcon plugins for Rust
RUN pip install --break-system-packages colcon-cargo colcon-ros-cargo

# Install workaround packages \
RUN apt update -qq && apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-example-interfaces \
    ros-$ROS_DISTRO-test-msgs \
    ros-$ROS_DISTRO-test-interface-files \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for neato_rust: libudev-dev
RUN apt update -qq && apt install -y --no-install-recommends \
      libudev-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /workspace/src
WORKDIR /workspace

RUN git clone -b $ROS_DISTRO https://github.com/ros2/common_interfaces.git src/common_interfaces
RUN git clone -b $ROS_DISTRO https://github.com/ros2/example_interfaces.git src/example_interfaces
RUN git clone -b $ROS_DISTRO https://github.com/ros2/rcl_interfaces.git src/rcl_interfaces
RUN git clone -b $ROS_DISTRO https://github.com/ros2/rosidl_core.git src/rosidl_core
RUN git clone -b $ROS_DISTRO https://github.com/ros2/rosidl_defaults.git src/rosidl_defaults
RUN git clone -b $ROS_DISTRO https://github.com/ros2/unique_identifier_msgs.git src/unique_identifier_msgs
RUN git clone https://github.com/ros2-rust/rosidl_rust.git src/rosidl_rust
RUN git clone https://github.com/ros2-rust/examples.git src/examples

RUN mkdir /workspace/src/ros2_rust
COPY . /workspace/src/neato_ros2_rust
RUN rm /workspace/src/neato_ros2_rust/Cargo.lock  # Somehow my laptop keeps generating such a file via RustRover that causes the build to fail

#RUN wget -O /workspace/src/ros2_rust/ros2_rust_${ROS_DISTRO}.repos https://raw.githubusercontent.com/ros2-rust/ros2_rust/d16172a97a13c6d4bef5912b2b99b2eb526a1915/ros2_rust_${ROS_DISTRO}.repos
#RUN vcs import src < src/ros2_rust/ros2_rust_${ROS_DISTRO}.repos

WORKDIR /workspace/src/neato_ros2_rust
RUN bash /opt/ros/${ROS_DISTRO}/setup.sh
WORKDIR /workspace
# RUN colcon build

#ENTRYPOINT ["/bin/bash"]