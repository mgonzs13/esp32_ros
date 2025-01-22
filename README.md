# esp32_ros

This repository demonstrates a simple use case of **micro-ROS** with an ESP32 board (specifically the ESP32-S3) to publish Wi-Fi scan data. Additionally, it provides a service to control the board's LED.

## Installation and Usage

### 1. Install micro-ROS

Follow the [official micro-ROS installation guide](https://micro.ros.org/docs/tutorials/core/first_application_linux/) to set up micro-ROS. Below is a summary of the steps:

```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep and pip
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

### 2. Install the Arduino IDE

Install the Arduino IDE using the following commands:

```bash
sudo apt install flatpak -y
sudo flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo
flatpak install flathub cc.arduino.arduinoide -y
```

### 3. Prepare micro-ROS for Arduino

1. Download the [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino/releases/tag/v2.0.7-humble) library and place it in `~/Arduino/libraries`.
2. Create custom interfaces for the `esp32_msgs` package:

```bash
# Copy the message package to the extra_packages directory
cp -r esp32_msgs ~/Arduino/libraries/micro_ros_arduino/extras/library_generation/extra_packages

# Create the code for Arduino
cd ~/Arduino/libraries/micro_ros_arduino
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble

# Optional: Copy the ESP32 library for the ESP32-S3 board
cd ~/Arduino/libraries/micro_ros_arduino/src
cp -r esp32 esp32s3

# Modify the colcon.meta file to use the parameter server in micro_ros_arduino
# Take a look at the provided colcon.meta file in this repository
cd ~/Arduino/libraries/micro_ros_arduino/extras/library_generation/extra_packages
nano colcon.meta
```

### 4. Configure the Arduino IDE

1. Open the Arduino IDE.
2. Install the **Adafruit_NeoPixel** library via `Tools > Manage Libraries`.
3. Install the **ESP32 board support package** via `Tools > Board > Board Manager`.
4. Select the correct port (`/dev/ttyACM0`) and the board (`ESP32S3 DEV Module`).

### 5. Load the Code and Run the micro-ROS Agent

1. Load the code from the `src` directory onto the ESP32 board using the Arduino IDE.
2. Start the micro-ROS agent:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### 6. Verify the Setup

- Check the `/wifi_scan` topic:

```bash
ros2 topic echo /wifi_scan
```

- Test the `/set_led_color` service:

```bash
ros2 service call /set_led_color esp32_msgs/srv/SetLedColor "{'color': {'r': 1, 'g': 0, 'b': 1}}"
```

## Additional Notes

- The `esp32_msgs` package defines the custom message and service types used in this project.
- Ensure all dependencies are properly installed to avoid build or runtime issues.

---

This project showcases the seamless integration of an ESP32-S3 board with ROS 2 using micro-ROS, providing an excellent starting point for robotics projects requiring lightweight and efficient communication between devices.
