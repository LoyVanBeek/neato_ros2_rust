ARG ROS_DISTRO=jazzy

FROM ros2_rust:latest-${ROS_DISTRO}
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

RUN mkdir /workspace/src
RUN mkdir /workspace/src/ros2_rust
COPY . /workspace/src/neato_ros2_rust

RUN wget -O /workspace/src/ros2_rust/ros2_rust_${ROS_DISTRO}.repos https://raw.githubusercontent.com/ros2-rust/ros2_rust/d16172a97a13c6d4bef5912b2b99b2eb526a1915/ros2_rust_${ROS_DISTRO}.repos
RUN vcs import src < src/ros2_rust/ros2_rust_${ROS_DISTRO}.repos

WORKDIR /workspace/src/neato_ros2_rust
RUN cargo build

ENTRYPOINT ["/bin/bash"]