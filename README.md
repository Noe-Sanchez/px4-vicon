# Companion Computer too PX4

This repository has the ROS 2 packages that manage the communication between the Companion Computer (RPi4 or Jetson for example) and the PX4-Autopilot flight computer (Pixhawk 6C for example).


### How to run it
git clone the repository and source the ros distribution, as well as the install folder
```
source install/setup.bash 
```
Run the agent in the Companion Computer

```
sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

> [!IMPORTANT]
> Check that your Companion Computer can communicate at the specific baud rate, as well set the TELEM2 baud rate Parameter in the PX4 Configuration

Run the ros2 file called viconpup.cpp
```
ros2 run topx4 viconpub
```
