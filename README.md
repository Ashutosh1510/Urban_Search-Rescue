# Urban_Search-Rescue

## Objective

This project is inspired by the challenge of autonomous robotics for Urban Search and Rescue (US&R). In US&R after a disaster occurs, a robot is used to explore an unknown environment, such as a partially collapsed building, and locates trapped or unconscious human victims of the disaster. The robot builds a map of the collapsed building as it explores, and places markers in the map of where the victims are located. This map is then given to trained First Responders who use it to go into the building and rescue the victims.  
• Instead of simulated victims, we use ArUco markers (squared ﬁducial markers).
• We will use a turtlebot (called explorer) to use a map of the building to find victims
(markers). We will then use another turlebot (called follower) to fetch the victims.

## Build the Package

```
source /opt/ros/noetic/setup.bash
git clone https://github.com/Ashutosh1510/Urban_Search-Rescue
```
Installing the required packages.
- The `script` folder contains install.bash to install required packages.

```
cd Urban_Search-Rescue-main/script
sudo chmod a+rwx install.bash
./install.bash
```
Building the package

```
catkin build
```

## Launch the package

cd to the workspace

```
source devel/setup.bash
roslaunch final_project multiple_robots.launch
```

Open an another terminal, cd to the workspace

```
source devel/setup.bash
rosrun final_project final_project_node
```


