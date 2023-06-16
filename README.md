# rrt_star_global_planner
## Please install this packages:

- gazebo: 'sudo apt-get install ros-noetic-gazebo-*'
- gmapping: 'sudo apt-get install ros-noetic-gmapping*'
- robot state: 'apt-get install ros-noetic-robot-state-*'
- joint : 'sudo apt-get install ros-noetic-joint-*'
- move base : 'sudo apt-get install ros-noetic-move-base*'
- navigation : 'sudo apt-get install ros-noetic-nav*'
- local planner : 'sudo apt-get install ros-noetic-dwa-local-planner'
- teb : 'sudo apt-get install ros-noetic-teb*'
- costmap : 'sudo apt-get install ros-noetic-costmap-*'
- controller : 'sudo apt-get install ros-noetic-controller-*'
- husky : 'sudo apt-get install ros-noetic-husky*'

## Installation

1. Create a Catkin workspace (if not already present)
  ```bash
  $ mkdir -p catkin_ws/src'''
2. Change directory to the source space (`src`) of your Catkin workspace
  ```bash
  $ cd catkin_ws/src
  ```
3. Clone this repository:
  ```bash
  $ git clone husky, rrt_star_global_planner, trajectory_details_recorder
  ```
4. Change directory back to the Catkin workspace:
  ```bash
  $ cd ..
  ```
5. Build the packages:
  ```bash
  $ catkin_make'''


## Usage:

1. Run the program:
  Ctrl+Alt+T
  cd catkin_ws 
  source devel/setup.bash
2. Run :
$ roslaunch husky_gazebo husky_playpen.launch  
$ roslaunch husky_viz view_robot.launch
$ roslaunch husky_navigation gmapping_demo.launch
$ rosrun trajectory_details_recorder make_plan.py
    
 
## RRT configuration
1. Open folder catkin_ws/src/husky/husky_navigation/config
Di dalam folder tersebut, Anda dapat menemukan planner.yaml
Di dalam planner.yaml, ada RRTStartPlanner yang di bawahnya adalah parameter yang berkaitan
2. method = 0 adalah RRT* dan method = 1 adalah RRT method = 2

## Kode yang mengimplementasikan algoritma RRT ada di:
catkin_ws/src/rt_star_global_planner/src/rrt_star.cpp

## Setting titik tujuan
catkin_ws/src/trajectory_details_recorder/script/smake_plan.py



