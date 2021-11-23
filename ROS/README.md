
# ROS packages and catkin

`panda_capacity` package will calculate is an example implementation of the task space capacity calculation for for Franka Emika Panda robot in a form of a catkin ros package.

It uses the library KDL for reading the robot urdf ad it depends on 
- hrl-kdl package (https://gitlab.inria.fr/auctus-team/people/antunskuric/ros_nodes/hrl-kdl)
- franka_description package (https://gitlab.inria.fr/auctus/panda/torque-qp-extended/franka_description)

The three packages are shown in the `packages` folder. And you can find an example catkin workspace in the `catkin_ws` forlder.

## Create your won catkin workspace

Then create new catkin workspace:
```shell
mkdir ~/capacity_ws && cd ~/capacity_ws/
mkdir src && cd src
```
Then you can copy the folders from `ROS/packages` into the `capacity_ws/src` folder for example:
```shell
cp -r ~/polytope_vertex_search/ROS_packages/* .
```

Finally you can build the workspace
```shell
cd ..
catkin_make # catkin build
```
And you should be ready to go!



## Dependancies
For visualizing the polytopes in RVIZ you will need to install the [jsk-rviz-plugin](https://github.com/jsk-ros-pkg/jsk_visualization)

```sh
sudo apt install ros-*-jsk-rviz-plugins # melodic/kinetic... your ros version
```

And you'll need to install the pip package `pycapacity`
```
pip install pycapacity
```
