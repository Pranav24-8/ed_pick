# Pick and Place Pipeline using ROS Environment Descriptor

This is a pick and place pipeline which uses the ROS Environment Descriptor package created by TU Eindhoven which can be found [here](https://github.com/tue-robotics/ed).

## Using the package

To use this package first follow the instruction in the Environment Descriptor package of TU Eindhoven up until reaching the part of querying the world model. Once this is possible, then the Pick and Place pipeline in this package can be used.

Clone this repository into the catkin workspace.

```bash
git clone https://github.com/Pranav24-8/ed_pick.git
```
Build the workspace.

Once the ED package is running, use the following command to run the pick and place pipeline.

```bash
roslaunch ed_pick ed_pick.launch
```
This package was tested on PAL Robotics Tiago robot.

## External Dependencies

ROS Environment Descriptor
