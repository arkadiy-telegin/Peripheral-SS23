## Changes to make it work with pointcloud 
+ set pointcloud_enable to true 
+ set color_width to 640
+ set color_height to 320

## Things to try
1. make it work with ROS2 Humble
2. two cameras at once
3. try copying realsense repo instead of cloning -> make changes outside of docker image

## Following launch file works:

### Lower latency achieved with following launch file
```
# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch realsense2_camera node."""
import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'unite_imu_method',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},

                           {'name': 'depth_module.profile',         'default': '424,240,15', 'description': 'depth module profile'},                           
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.profile',           'default': '424,240,6', 'description': 'color image width'},
                           {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'enable_fisheye1',              'default': 'true', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',              'default': 'true', 'description': 'enable fisheye2 stream'},
                           {'name': 'enable_confidence',            'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps',                     'default': '0', 'description': "''"},                           
                           {'name': 'accel_fps',                    'default': '0', 'description': "''"},                           
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "''"},                           
                           {'name': 'enable_accel',                 'default': 'false', 'description': "''"},                           
                           {'name': 'enable_pose',                  'default': 'true', 'description': "''"},                           
                           {'name': 'pose_fps',                     'default': '200', 'description': "''"},                           
                           {'name': 'pointcloud.enable',            'default': 'true', 'description': ''}, 
                           {'name': 'pointcloud.stream_filter',     'default': 'RS2_STREAM_COLOR', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync',                  'default': 'true', 'description': "''"},                           
                           {'name': 'align_depth.enable',           'default': 'true', 'description': "''"},                           
                           {'name': 'colorizer.enable',             'default': 'false', 'description': "''"},
                           {'name': 'clip_distance',                'default': '-2.', 'description': "''"},                           
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},                           
                           {'name': 'initial_reset',                'default': 'false', 'description': "''"},                           
                           {'name': 'allow_no_texture_points',      'default': 'true', 'description': "''"},                           
                           {'name': 'pointcloud.ordered_pc',        'default': 'false', 'description': ''},
                           {'name': 'publish_tf',                   'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': '[double] rate in Hz for publishing dynamic TF'},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'decimation_filter.enable',     'default': 'true', 'description': 'Rate of publishing static_tf'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'depth_module.exposure.1',      'default': '7500', 'description': 'Depth module first exposure value. Used for hdr_merge filter'},
                           {'name': 'depth_module.gain.1',          'default': '16', 'description': 'Depth module first gain value. Used for hdr_merge filter'},
                           {'name': 'depth_module.exposure.2',      'default': '1', 'description': 'Depth module second exposure value. Used for hdr_merge filter'},
                           {'name': 'depth_module.gain.2',          'default': '16', 'description': 'Depth module second gain value. Used for hdr_merge filter'},
                           {'name': 'depth_module.exposure',        'default': '8500', 'description': 'Depth module manual exposure value'},
                           {'name': 'depth_module.hdr_enabled',     'default': 'false', 'description': 'Depth module hdr enablement flag. Used for hdr_merge filter'},
                           {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'hdr_merge.enable',             'default': 'false', 'description': 'hdr_merge filter enablement flag'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, *args, **kwargs):
    _config_file = LaunchConfiguration("config_file").perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)
    log_level = 'info'
    # Realsense
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return [
            launch_ros.actions.Node(
                package='realsense2_camera',
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters=[set_configurable_parameters(configurable_parameters)
                            , params_from_file
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                )
            ]
    else:
        return [
            launch_ros.actions.Node(
                package='realsense2_camera',
                namespace=LaunchConfiguration("camera_name"),
                name=LaunchConfiguration("camera_name"),
                executable='realsense2_camera_node',
                parameters=[set_configurable_parameters(configurable_parameters)
                            , params_from_file
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                emulate_tty=True,
                )
        ]
    
def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function=launch_setup)
    ])
```


# Docker for Intel Realsense cameras on ROS 2

Author: [Tobit Flatscher](https://github.com/2b-t) (June 2022 - March 2023)

[![Build](https://github.com/2b-t/realsense-ros2-docker/actions/workflows/build.yml/badge.svg)](https://github.com/2b-t/realsense-ros2-docker/actions/workflows/build.yml) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)



## 0. Overview
This repository contains a Docker and all the documentation required to launch an [Intel Realsense camera](https://www.intel.co.uk/content/www/uk/en/architecture-and-technology/realsense-overview.html) with the [Robot Operating System ROS 2](https://docs.ros.org/en/humble/index.html).

## 1. Creating a Docker
There are two different approaches for creating a Docker for a Realsense camera, one uses existing **Debian packages** while the other performs a full **compilation from source**. It is then important to mount `/dev` as a volume so that the Docker can access the hardware.

In the `docker-compose.yml` this is done with the options:

```yaml
    volumes:
      - /dev:/dev
    device_cgroup_rules:
      - 'c 81:* rmw'
      - 'c 189:* rmw'
```

### 1.1 Installation from Debian packages
The relevant packages for `amd64` and most distributions can be installed from Debian packages. This is significantly simpler and less error prone than a full compilation from source but corresponding Debian packages might not be available for all Ubuntu versions and are sadly currently not available for `arm64`. **In the case of an `arm64` architecture you will have to go for a compilation from source as described in the next section.** The corresponding Dockerfile for `amd64` can be found below. It is based on the installation guides for [`librealsense`](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) and [its ROS wrapper](https://github.com/IntelRealSense/realsense-ros/tree/ros2-beta).

```Dockerfile
FROM ros:humble-perception

ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common

# Install dependencies: See https://github.com/IntelRealSense/realsense-ros/tree/ros2-development
# Librealsense: See https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
RUN apt-get update -y \
 && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
 && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
 && apt-get install -y \
    librealsense2-dbg \
    librealsense2-dev \
    librealsense2-dkms \
    librealsense2-utils

RUN apt-get update -y \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
 && mkdir src \
 && cd src \
 && git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development \
 && cd .. \
 && apt-get install -y python3-rosdep \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rm /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} --skip-keys=librealsense2 -y \
 && colcon build

ARG DEBIAN_FRONTEND=dialog
```

### 1.2 Installation from source
The installation from source is slightly more involved but more general. It might work with ROS 2 distributions before the support is officially added (as used to be the case for ROS Humble for quite a while) as well as with **architectures for which no Debian packages are available such as `arm64`** (e.g. the Nvidia Jetson family, see [issue #1](https://github.com/2b-t/realsense-ros2-docker/issues/1#)). In order to optimise memory usage a [multi-stage build](https://docs.docker.com/develop/develop-images/multistage-build/) is performed. The Dockerfile is inspired by the [Dockerfile of `librealsense`](https://github.com/IntelRealSense/librealsense/blob/master/scripts/Docker/Dockerfile) contributed by community members:

```Dockerfile
ARG BASE_IMAGE=ros:humble-perception

# The following steps are based on the offical multi-stage build: https://github.com/IntelRealSense/librealsense/blob/master/scripts/Docker/Dockerfile
#################################
#   Librealsense Builder Stage  #
#################################
FROM $BASE_IMAGE as librealsense-builder

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \    
    curl \
    python3 \
    python3-dev \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/src
# Get the latest tag of remote repository: https://stackoverflow.com/a/12704727
# Needs to be a single command as ENV can't be set from Bash command: https://stackoverflow.com/questions/34911622/dockerfile-set-env-to-result-of-command
RUN export LIBRS_GIT_TAG=`git -c 'versionsort.suffix=-' \
                         ls-remote --exit-code --refs --sort='version:refname' --tags https://github.com/IntelRealSense/librealsense '*.*.*' \
                         | tail --lines=1 \
                         | cut --delimiter='/' --fields=3`; \
    export LIBRS_VERSION=${LIBRS_VERSION:-${LIBRS_GIT_TAG#"v"}}; \
    curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v${LIBRS_VERSION} -o librealsense.tar.gz; \
    tar -zxf librealsense.tar.gz; \
    rm librealsense.tar.gz; \
    ln -s /usr/src/librealsense-${LIBRS_VERSION} /usr/src/librealsense

RUN cd /usr/src/librealsense \
 && mkdir build && cd build \
 && cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \    
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ../ \
 && make -j$(($(nproc)-1)) all \
 && make install
 
 ENV DEBIAN_FRONTEND=dialog

######################################
#   librealsense Base Image Stage    #
######################################
FROM ${BASE_IMAGE} as librealsense

SHELL ["/bin/bash", "-c"]

COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
ENV PYTHONPATH=${PYTHONPATH}:/usr/local/lib

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y --no-install-recommends \	
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# The following steps are based on: https://github.com/IntelRealSense/realsense-ros/tree/ros2-development
ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}
RUN apt-get update -y \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
 && mkdir src \
 && cd src \
 && git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development \
 && cd .. \
 && apt-get install -y python3-rosdep \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rm /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} --skip-keys=librealsense2 -y \
 && colcon build

ENV DEBIAN_FRONTEND=dialog

CMD [ "rs-enumerate-devices", "--compact" ]
```

## 2. Launching
Allow the container to display contents on your host machine by typing

```bash
$ xhost +local:root
```

Then build the Docker container with

```shell
$ docker compose -f docker-compose-gui.yml build
```
or directly with the [`devcontainer` in Visual Studio Code](https://code.visualstudio.com/docs/devcontainers/containers). For Nvidia graphic cards the file `docker-compose-gui-nvidia.yml` in combination with the [`nvidia-container-runtime`](https://nvidia.github.io/nvidia-container-runtime/) has to be used instead.
After it is done building **connect the Realsense**, start the container

```shell
$ docker compose -f docker-compose-gui.yml up
```
and see if you can **detect it from inside the Docker** by typing inside the Docker
```shell
$ rs-enumerate-devices --compact
```
Then continue to launch the proprietary visualisation applet
```shell
$ realsense-viewer
```
Turn on the camera inside the application, see if you can see a three-dimensional image. Finally we can **launch the ROS 2 wrapper**
```shell
$ ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```
and in another terminal the ROS visualiser Rviz
```shell
$ ros2 run rviz2 rviz2
```
In Rviz you can then display the topics published by the Realsense. For the list of available topics use
```shell
$ ros2 topic list
```
in combination with
```shell
$ ros2 topic info <topic_name>
```
to find out what display type has to be selected in Rviz.

![Rviz2 preview](./media/preview.png)



## 3. Debugging

The Intel Realsense driver has several serious flaws/bugs. Probably the main one is that it is **closely connected to the [kernel version of the Linux operating system](https://github.com/IntelRealSense/librealsense/issues/9360)**. If the Dockerfile above do not work then you are likely unlucky and it is an incompatible version of the kernel of your host system and you will either have to [downgrade your kernel](https://linuxhint.com/install-linux-kernel-ubuntu/) or switch to another Ubuntu version that is officially supported. Furthermore from time to time the software will give you cryptic error messages. For some restarting the corresponding software component might help, for others you will find a fix googling and with others you will have to learn to live.
The Realsense is **pretty [picky about USB 3.x cables](https://github.com/IntelRealSense/librealsense/issues/2045)**. If your camera is detected via `rs-enumerate-devices`, you can see it `realsense-viewer` but can't output its video stream, then it might be that your cable lacks the bandwidth. Either you can try to turn down the resolution of the camera in the `realsense-viewer` or switch cable (preferably to one that is [already known to work](https://community.intel.com/t5/Items-with-no-label/long-USB-cable-for-realsense-D435i/m-p/694963)).

