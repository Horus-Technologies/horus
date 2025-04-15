# Start from Ubuntu 22.04 (Jammy Jellyfish)
FROM ubuntu:22.04

# Set DEBIAN_FRONTEND to noninteractive to avoid prompts
ARG DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Set shell for RUN commands
SHELL ["/bin/bash", "-c"]

# === Basic Setup and Locales ===
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# === Add ROS 2 APT Repository ===
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# === Install ROS 2 Humble Desktop, Dev Tools, and Build Essentials ===
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-dev-tools \
    # Common build tools needed for your other steps
    git \
    cmake \
    build-essential \
    pkg-config \
    sed \
    python3-pip \
    python3-rosdep \
    python3-venv \
    # Add any other essential packages needed early
    && apt-get upgrade -y \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# === Initialize rosdep ===
RUN rosdep init || echo "rosdep already initialized" \
    && rosdep update

# === Python Setup ===
# Upgrade pip and install specific numpy version (Warning: NumPy 2.x might cause issues with Humble)
RUN pip3 install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir wheel numpy==1.24.2

# Set home directory for subsequent operations
WORKDIR /root

# Create main project directory
RUN mkdir horus
WORKDIR /root/horus

# === Install Pangolin Dependencies ===
# Installs packages corresponding to the 'recommended' option in install_prerequisites.sh
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools often needed (some overlap with below, ensures they are present)
    git cmake build-essential pkg-config \
    # === PKGS_REQUIRED from script ===
    libgl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libc++-dev \
    libepoxy-dev \
    libglew-dev \
    libpython3-dev \
    libeigen3-dev \
    g++ \
    ninja-build \
    # === PKGS_RECOMMENDED from script ===
    libjpeg-dev \
    libpng-dev \
    catch2 \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    # Required for python bindings if needed (was in PKGS_ALL, but often desired)
    # Add python3-dev if you didn't install it earlier or need Pangolin python bindings
    # python3-dev
    # Cleanup
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# === Pangolin Installation ===
WORKDIR /root/horus
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git \
    && cd Pangolin \
    # DO NOT RUN ./scripts/install_prerequisites.sh recommended
    && cmake -B build \
    && cmake --build build -j$(nproc) \
    && cmake --install build \
    # Clean up source directory
    && cd .. && rm -rf Pangolin \
    # Update linker cache
    && ldconfig

# Add /usr/local/lib to library path persistently
ENV LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
# Optional: Set WAYLAND_DISPLAY if running GUI apps
ENV WAYLAND_DISPLAY=0

# === ORB-SLAM3 ROS 2 Wrapper Installation ===
WORKDIR /root
RUN mkdir -p ros2_test/src
WORKDIR /root/ros2_test/src
RUN git clone https://github.com/Mechazo11/ros2_orb_slam3.git
WORKDIR /root/ros2_test

# Install ORB-SLAM3 dependencies using rosdep
# Source ROS setup before running rosdep install and colcon build
# Explicitly install libssl-dev for libcrypto and run apt-get update first
RUN apt-get update && apt-get install -y --no-install-recommends libssl-dev \
    && source /opt/ros/humble/setup.bash \
    && rosdep install -r --from-paths src --ignore-src -y --rosdistro humble \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Build ORB-SLAM3 workspace
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

# === PX4 Autopilot Installation ===
WORKDIR /root/horus
# Clone PX4 Autopilot
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
# Run the PX4 setup script (this is complex and might fail)
# Ensure python3-venv is installed as the script might need it
RUN apt-get update && apt-get install -y python3-venv && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN cd PX4-Autopilot \
    && Tools/setup/ubuntu.sh --no-interaction || echo "PX4 setup script finished (check logs for errors)"
# Build PX4 SITL target
WORKDIR /root/horus/PX4-Autopilot
RUN make px4_sitl
# Install required python packages globally
RUN pip3 install --no-cache-dir -U empy==3.3.4 pyros-genmsg setuptools

# === Micro-XRCE-DDS Agent Installation ===
WORKDIR /root/horus
RUN git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent \
    && sed -i 's/2\.12\.x/2.12.1/g' CMakeLists.txt \
    && mkdir build && cd build \
    && cmake .. \
    && make \
    && make install \
    # Update linker cache specifically for /usr/local/lib
    && ldconfig /usr/local/lib/ \
    # Clean up source directory
    && cd ../.. && rm -rf Micro-XRCE-DDS-Agent

# === PX4 ROS Communication Packages Installation ===
WORKDIR /root/horus
RUN mkdir -p ws_px4/src/
WORKDIR /root/horus/ws_px4/src
RUN git clone https://github.com/PX4/px4_msgs.git \
    && git clone https://github.com/PX4/px4_ros_com.git
WORKDIR /root/horus/ws_px4

# Pin packaging library to a version compatible with ROS Humble build tools
RUN pip3 install --no-cache-dir packaging==22.0

# Build the PX4 ROS workspace
RUN source /opt/ros/humble/setup.bash \
    && colcon build

# === Final Setup ===
# Copy the entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Set the default command to bash
CMD ["bash"]

# Set final working directory (optional)
WORKDIR /root

# Inform user build is complete
RUN echo "ROS 2 Humble, PX4, ORB-SLAM3 environment build complete."
