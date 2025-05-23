
# Full Setup Guide: Running SumoWare with Autoware on Windows 11 (WSL2 + Docker)

---

## 🖥️ Host System
- Windows 11
- WSL2 (Ubuntu 22.04)
- Python 3.10
- Docker Desktop (WSL2 integration enabled)
- VcXsrv (for RViz2 GUI)

---

## 🧰 Step-by-Step Setup

### ✅ Step 1: Install WSL2 + Ubuntu 22.04

In PowerShell (Admin):

```powershell
wsl --install -d Ubuntu-22.04
```

Reboot when prompted. Launch Ubuntu, create your UNIX user and password.

---

### ✅ Step 2: System Setup in Ubuntu

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl wget git lsb-release gnupg2 software-properties-common
```

---

### ✅ Step 3: Install ROS 2 Humble

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc > /dev/null
sudo sh -c 'echo "deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/ros.asc] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update
sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### ✅ Step 4: Install ROS Tools and Python Deps

```bash
sudo apt install -y python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update

pip3 install --upgrade pip
pip3 install requests==2.31.0 transforms3d commonroad-scenario-designer empy==3.3.4
```

---

### ✅ Step 5: Install SUMO

```bash
sudo apt install -y sumo sumo-tools sumo-doc
echo 'export SUMO_HOME="/usr/share/sumo"' >> ~/.bashrc
source ~/.bashrc
```

---

### ✅ Step 6: Clone and Configure SumoWare

```bash
git clone --recurse-submodules https://github.com/TUM-VT/SumoWare.git
cd SumoWare
git config submodule.src/tier4_autoware_msgs.url https://github.com/tier4/tier4_autoware_msgs.git
git submodule update --init --recursive
```

---

### ✅ Step 7: Fix SetupTools Compatibility for Colcon

```bash
pip3 install setuptools==59.6.0
rm -rf build install log
```

---

### ✅ Step 8: Build Workspace

```bash
colcon build --symlink-install
```

If needed, you can skip sumoware first:

```bash
colcon build --symlink-install --packages-skip sumoware
colcon build --symlink-install --packages-select sumoware
```

---

### ✅ Step 9: Build SUMO Docker Image

```bash
docker build ./sumo_docker -t sumo_docker
```

---

### ✅ Step 10: Launch SumoWare Scenario Controller

```bash
source ~/SumoWare/install/local_setup.bash

ros2 run sumoware scenario_controller --ros-args \
  -p use_sim_time:=true \
  -p scenario_file:="./example_inputs/scenario_params.json" \
  -p map_file:="./example_inputs/map.xodr" \
  -p project_absolute_path:="$PWD" \
  -p gpu_support:=false
```

---

## 🚗 Autoware Runtime Setup (Docker)

### ✅ Step 11: Run Autoware Container

```bash
docker run -it --rm \
  --net=host \
  --privileged \
  -v $PWD/example_inputs:/autoware_map \
  ghcr.io/autowarefoundation/autoware:universe-cuda-20241010-amd64 \
  bash
```

### ✅ Step 12: Source and Launch

```bash
source /opt/autoware/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/autoware_map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  rviz:=false
```

---

## 🖼️ Optional: GUI Support via RViz

1. Install VcXsrv from https://sourceforge.net/projects/vcxsrv/
2. Launch with: Multiple windows → No client → Disable access control
3. Find host IP in WSL:
```bash
cat /etc/resolv.conf | grep nameserver
```
4. Launch Docker with:
```bash
-e DISPLAY=<HOST_IP>:0
```
5. Inside container:
```bash
apt install -y libxcb-xinerama0
source /opt/autoware/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml ...
```

---

## ✅ Success!

At this point:
- SumoWare launches scenarios
- Autoware receives commands
- SUMO and RViz visualize ego + environment

---

## 🖼️ VcXsrv Setup for Visualizing RViz2 (Optional)

To display GUI applications like RViz2 from inside your Docker container or WSL2, follow these steps:

### ✅ Step 1: Download and Install VcXsrv

1. Go to: [https://sourceforge.net/projects/vcxsrv/](https://sourceforge.net/projects/vcxsrv/)
2. Click the green **"Download"** button.
3. Run the installer and follow the default installation steps:
   - Install for all users
   - Default components
   - Finish installation

---

### ✅ Step 2: Launch X Server with XLaunch

1. Press the **Windows key** and search for **"XLaunch"**.
2. Follow the wizard:
   - **Display settings** → Select **“Multiple windows”** → Click **Next**
   - **Client startup** → Select **“Start no client”** → Click **Next**
   - **Extra settings**:
     - ✅ Enable **"Disable access control"**
     - Leave other options as default → Click **Next**
   - Click **Finish**

VcXsrv is now running. It will sit in your system tray with a small black-and-white "X" icon.

---

### ✅ Step 3: Configure Docker to Use Your Display

1. In WSL2, run:

```bash
cat /etc/resolv.conf | grep nameserver
```

2. Note the IP address, e.g., `172.25.112.1`

3. When launching Docker, pass the DISPLAY variable:

```bash
docker run -it --rm \
  --net=host \
  --privileged \
  -e DISPLAY=172.25.112.1:0 \
  -v $PWD/example_inputs:/autoware_map \
  ghcr.io/autowarefoundation/autoware:universe-cuda-20241010-amd64 \
  bash
```

Replace `172.25.112.1` with the IP address from your machine.

---

### ✅ Step 4: Install Missing Libraries (if needed)

Inside the Docker container:

```bash
apt update
apt install -y libxcb-xinerama0
```

---

### ✅ Step 5: Launch RViz2 from Autoware

```bash
source /opt/autoware/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/autoware_map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

RViz2 should now open in a window on your Windows desktop via VcXsrv.
