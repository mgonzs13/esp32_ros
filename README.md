# esp32_ros

This repository present a simple use of micro-ROS with a esp32 board (specifically a esp32-s3 board) to publish wifi scan data. Besides, it also provides a service to control the board led.

## Installation and Usage

1. First of all, you have to install micro-ROS. Follow the following instructions that you can find in the micro-ROS [page](https://micro.ros.org/docs/tutorials/core/first_application_linux/).

```shell
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

2. Then you have to install the Arduino IDE:

```shell
sudo apt install flatpak -y
sudo flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo
flatpak install flathub cc.arduino.arduinoide -y
```

3. Now lets prepare micro-ROS for Arduino. Download [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino/releases/tag/v2.0.7-humble) and put it in `~/Arduino/libraries`.

4. The next step consists on creating the custom interfaces of `esp32_msgs` for micro_ros_arduino. You will also need to put this package in you ROS 2 workspace.

```shell
# Copy the msg package to the extra_packages directory
cp -r esp32_msgs ~/Arduino/libraries/micro_ros_arduino/extras/library_generation/extra_packages

# Create the code for Arduino
cd ~/Arduino/libraries/micro_ros_arduino
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble

# Optional: copy esp32 for esp32s3
cd ~/Arduino/libraries/micro_ros_arduino/src
cp -r esp32 esp32s3

# Modify the colcon.meta to use parameter server in micro_ros_arduino
# Take a look to the provided colcon.meta file in this repo
cp ~/Arduino/libraries/micro_ros_arduino/extras/library_generation/extra_packages
nano colcon.meta
```

5. Time to configure the Arduino IDE. Open the Arduino IDE and install the `Adafruit_NeoPixel` from `Tools > Manage Libraries`. Then install the `esp32` board from `Tools > Board > Board Manager`. Choose your port (`/dev/ttyACM0`) and the board (`ESP32S3 DEV Module`).

6. Finally, load the code in `src` and run the micro-ROS agent:

```shell
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

7. Check the `/wifi_scan` topic and the `/set_led_color` service:

```shell
ros2 topic echo /wifi_scan
ros2 service call /set_led_color esp32_msgs/srv/SetLedColor "{'color': {'r': 1, 'g': 0, 'b': 1}}"
```
