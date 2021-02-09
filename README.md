# ROS Wrapper for TIS Camera

## Camera Driver Installation Guide

The camera driver has two components

- `tiscamera` which is the core driver
- `tiscamera-dutil` which contains many additional useful features, such as tonemapping

## Step 1/3: Install `tiscamera` Core Driver

For Intel-based processor, you may choose to install the official compiled **tiscamera** package from here: https://github.com/TheImagingSource/tiscamera/releases

**Note:**
- As of Jan 2021, we are using version 0.12.0
- For TX2 or Xavier, if cannot build from source, can try to install the official compiled tiscamera package  `tiscamera_0.12.0_arm64.deb` directly.


However, it is strongly recommanded to build from source, for the main tiscamera driver, especially on Nvidia arm64 platform.

### Build `tiscamera` From Source
0. Install `GStreamer` through apt first
1. Clone `https://github.com/TheImagingSource/tiscamera` and checkout the release version, for example `v-tiscamera-0.12.0`

    ``` bash
    git clone https://github.com/TheImagingSource/tiscamera.git
    git checkout v-tiscamera-0.12.0
    
2. Change CMakeList.txt `BUILD_TOOLS` to `ON`
3. (Jetson TX2) Install dependencies `gstreamer-1.0 libusb-1.0 libglib2.0 libgirepository1.0-dev libudev-dev libtinyxml-dev libzip-dev libnotify-dev`
4. Install python dependencies `python3-gi python3-pyqt5`
5. Uninstall existing apt package `sudo apt remove tiscamera`
6. use CMake build and install:
    ``` bash
    mkdir build && cd build
    cmake -DBUILD_ARAVIS=OFF ..
    make
    sudo make install
    ```

## Step 2/3: Install `tiscamera-dutils`

Also install the **dutils** package here `tiscamera-dutils_1.0.0.160`
https://github.com/chengguizi/tiscamera_ros/tree/master/sdk_debs

**Tip:**

To verify driver installation, connect the camera using usb cable, then run `tcam-capture`, and there should be an option for tonemapping showing up. Otherwise, the installation is unsuccessful.

Also, in `gst-inspect-1.0 tcambin`, the `dutil` should have default to be true.


## Step 3/3: Integrating with ROS: `tiscamera_ros`

The ROS driver could be found at https://github.com/chengguizi/tiscamera_ros/

Dependencies:
- mavlink (install by apt `ros-melodic-mavlink`)
- mavros (https://github.com/chengguizi/mavros, Feb 2021 Release)

There are two files important for configuration, in the launch folder
- the `device_list` file, which defines the camera string name, as well as serial number
  - `hardware_sync_mode` set to `none` if no hardware synchronisation is used
  - if hardware sync is used, it should set to `slave`
- the `param` file, which sets the exposure settings for all cameras

Tip: install missing ROS dependency with `rosdep install --from-paths src --ignore-src -r -y` (require python-rosdep package)

## Add `dialout` Group

Remember to enable serial access, by adding current user to `dialout` group:
```bash
sudo adduser nvidia dialout
```
## Get Started

After `catkin build tiscamera_ros`, the `snapimage` executable should reside in the `build\tiscamera_ros\tests` folder. It could be used to take snapshot images without running ROS.