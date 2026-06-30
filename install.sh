#!/bin/bash
set -e

# make sure to run this on a

# Check Ubuntu version
if [[ $(lsb_release -rs) != "22.04" ]]; then
    echo "ERROR: This script requires Ubuntu 22.04"
    exit 1
fi

WAUV_DIR=~/WAUV
REPO_DIR="$WAUV_DIR/WAUV_Stonefish"

echo "==> Installing system dependencies"
sudo apt update
sudo apt install -y \
    curl gnupg lsb-release build-essential cmake git \
    python3-pip \
    libglm-dev \
    libsdl2-dev \
    libfreetype-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    libegl1-mesa-dev \
    mesa-utils \
    doxygen \
    graphviz

python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install "setuptools<80"

# ros2 humble isntall
if ! command -v ros2 &> /dev/null; then
    echo "==> Installing ROS2 Humble"

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y \
        ros-humble-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool
else
    echo "==> ROS2 Humble already installed, skipping"
fi

if ! grep -q "ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

source /opt/ros/humble/setup.bash

# ros2 deps that stonefish needs
echo "==> Installing ROS2 packages"
sudo apt install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-tf-transformations \
    ros-humble-pcl-ros \
    python3-transforms3d \
    geographiclib-tools

sudo geographiclib-get-geoids egm96-5 || true

# stonefish install
mkdir -p "$WAUV_DIR"
cd "$WAUV_DIR"

if [ ! -d "stonefish" ]; then
    echo "==> Cloning libstonefish"
    git clone https://github.com/patrykcieslak/stonefish.git
else
    echo "==> libstonefish already cloned, skipping"
fi

cd "$WAUV_DIR/stonefish"

if [ ! -f "build/libstonefish.so" ] && [ ! -f "/usr/local/lib/libstonefish.so" ]; then
    echo "==> Building libstonefish"
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig
else
    echo "==> libstonefish already built, skipping"
fi

# ardupilot install
cd "$WAUV_DIR"

if [ ! -d "ardupilot" ]; then
    echo "==> Cloning ArduPilot"
    git clone https://github.com/ArduPilot/ardupilot.git
else
    echo "==> ArduPilot already cloned, skipping"
fi

cd "$WAUV_DIR/ardupilot"
git submodule update --init --recursive

Tools/environment_install/install-prereqs-ubuntu.sh -y

if ! grep -q "ardupilot/Tools/completion/completion.bash" ~/.bashrc; then
    echo "source ~/WAUV/ardupilot/Tools/completion/completion.bash" >> ~/.bashrc
fi

if ! grep -q "ardupilot/Tools/autotest" ~/.bashrc; then
    echo "export PATH=\$PATH:~/WAUV/ardupilot/Tools/autotest" >> ~/.bashrc
fi



echo "==> Building WAUV_Stonefish workspace"
cd "$REPO_DIR"

source /opt/ros/humble/setup.bash

# Initialize rosdep if needed
sudo apt install -y python3-rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

if ! grep -q "WAUV_Stonefish/install/setup.bash" ~/.bashrc; then
    echo "source ~/WAUV/WAUV_Stonefish/install/setup.bash" >> ~/.bashrc
fi

echo ""
echo "======================================"
echo " INSTALL COMPLETE (づ ◕‿◕ )づ"
source ~/.bashrc
echo " Run: source ~/.bashrc"
echo " Then: ros2 launch stonefish_bluerov2 bluerov2_sim.py"
echo " Dont forget to launch ardusub and mavros too appropriately"
echo "======================================"