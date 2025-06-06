# Universal_Robots_ROS2_Description

This repository contains description files and meshes for *Universal Robots* manipulators.

## Build status

ROS2 Distro | Branch | Build status | Released packages
:---------: | :----: | :----------: | :---------------:
**Humble** | [`humble`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble) | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__ur_description__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__ur_description__ubuntu_jammy_amd64__binary/) | [ur_description](https://index.ros.org/p/ur_description/#humble)
**Jazzy** | [`jazzy`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__ur_description__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__ur_description__ubuntu_noble_amd64__binary/) | [ur_description](https://index.ros.org/p/ur_description/#jazzy)
**Kilted** | [`rolling`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/rolling) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__ur_description__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__ur_description__ubuntu_noble_amd64__binary/) | [ur_description](https://index.ros.org/p/ur_description/#kilted)
**Rolling** | [`rolling`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/rolling) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__ur_description__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__ur_description__ubuntu_noble_amd64__binary/)| [ur_description](https://index.ros.org/p/ur_description/#rolling)

A more [detailed build status](ci_status.md) shows the state of all CI workflows inside this repo.
Please note that the detailed view is intended for developers, while the one here should give end
users an overview of the current released state.


## License
The [UR15 meshes](meshes/ur15), [UR20 meshes](meshes/ur20) and [UR30 meshes](meshes/ur30) constitutes “Graphical Documentation” the use of which is subject to and governed by our “[Terms and Conditions for use of Graphical Documentation](https://www.universal-robots.com/legal/terms-and-conditions/terms_and_conditions_for_use_of_graphical_documentation.txt)”.

Universal Robots' [Terms and Conditions for use of Graphical Documentation](https://www.universal-robots.com/legal/terms-and-conditions/terms_and_conditions_for_use_of_graphical_documentation.txt) do not fully comply with [OSI's definition of Open Source](https://opensource.org/osd/), but they do allow you to use, modify and share “Graphical Documentation”, including [UR15 meshes](meshes/ur15), [UR20](meshes/ur20) and [UR30](meshes/ur30) meshes, subject to certain restrictions.\
If you have any questions regarding this license or if this license doesn't fit your use-case, please contact [legal@universal-robots.com](mailto:legal@universal-robots.com).

All other content is licensed under the BSD-3-Clause license

## Structure of the repository

The most relevant files are:
  - `urdf/ur_macro.xacro` - macro file with UR-manipulator description. This file is usually included into external projects to visualize and configure UR manipulators properly. An example how to use this macro is in `urdf/ur.urdf.xacro` file.
  - `urdf/ur.ros2_control.xacro` - definition of manipulator's joints and interfaces for `ros2_control` framework.

## Testing description of a manipulator

To visualize the robot install this repository to you workspace and execute the following:

``` bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
```

To test other descriptions change the `ur_type` argument.

## Package / Description structure
This package uses one description for all robots. The different robot variants are configured using
four configuration files. These files can also be changed for further customizing a description.

![urdf structure](doc/structure.svg)

Basically, the description can be modified using configuration values stored in four files:
 - `config/urXX/default_kinematics.yaml` - This contains the calibration values as they can be
   extracted from the robot. Changing these values with the one extracted from a real robot will
   result in a description matching the real robot exactly (w.r.t the `tool0` frame). It is highly
   recommended to use matching kinematic values in real-world applications.
 - `config/urXX/joint_limits.yaml` - If you'd like to further restrict the robot's joint limits,
   these limits can be modified there.
 - `config/urXX/physical_parameters.yaml` - Everything regarding physics simulation parameters
   (e.g. inertia poses and values) can be tuned here
 - `config/urXX/visual_parameters.yaml` - Some users change certain visual aspects, e.g. replacing
   the cap on the wrist_3_link. This config file specifies which meshes (both, visual and collision)
   should be used.

The four configuration files have to be passed to `ur_macro.urdf` (more specific to the macro
defined in that file) which is done inside the `ur.urdf.xacro`. Contents of the files are parsed
inside `ur_common.xacro`.

Arguments that have to be passed to the main `ur.urdf.xacro` file are:
 - kinematics_params - Filename to the `default_kinematics.yaml` (or equivalent specific kinematics) file
 - joint_limit_params - Filename to the `joint_limits.yaml` file
 - physical_params - Filename to the `physical_parameters.yaml` file
 - visual_params - Filename to the `visual_params.yaml` file

The launchfile `launch/view_ur.launch.py` abstracts these four parameters to one `ur_type` argument
which will basically replace the `urXX` part of the paths as shown in the picture above.

## Creating your own description including this description
In real-world applications you will most probably have a more complex description consisting of more
objects than just the robot. It is recommended to create a separate ROS package containing this
particular description. Inside this description you could also store your robot-specific kinematics
parameters file.

As mentioned above, see the `urdf/ur.urdf.xacro` file as an example to integrate a UR robot into
your scene description. Basically, you could create a copy of that file and extend it with the
modifications from your specific scene.
