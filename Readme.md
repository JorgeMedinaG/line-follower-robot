# Introduction
This repository contains the implementation of a line-following algorithm for a four-wheel robot integrated with ROS. The robot uses a differential motion model and relies on a camera to capture the line it needs to follow, adjusting its movement based on the line's position.

## How to launch
Please execute following command to run our project.
```
docker/build_docker.sh
docker/run_docker.sh
bash line_follower_ws/build.bash 
roslaunch line_follower course.launch
python3 line_follower_ws/follow_line.py
```

# Overview
Following folder tree shows how our repo structures
```
.
├── docker
│   ├── 10_nvidia.json
│   ├── build_docker.sh
│   ├── Dockerfile
│   ├── install_docker.sh
│   ├── into_docker.sh
│   ├── README.md
│   ├── req
│   │   ├── build.bash
│   │   └── workspace_building.bash
│   └── run_docker.sh
├── docs
│   
├── line_follower_ws
│   ├── build.bash
│   ├── devel
│   ├── follow_line.py
│   └── src
│       ├── CMakeLists.txt
│       ├── line_follower
│       │   ├── CMakeLists.txt
│       │   ├── course.material
│       │   ├── course.png
│       │   ├── course.world
│       │   ├── launch
│       │   │   ├── course.launch
│       │   │   └── gazebo_motor.launch
│       │   └── package.xml
│       └── quad_wheel_robot_6
│           ├── CMakeLists.txt
│           ├── config
│           │   ├── diffdrive.yaml
│           │   ├── gazebo_ros_control_params.yaml
│           │   └── joints.yaml
│           ├── meshes
│           │   └── base_link.STL
│           ├── package.xml
│           └── urdf
│               └── quad_wheel_robot_6_motor.urdf
└── Readme.md
```

Folder annotation:
- `docker`: Containing docker setup for this implementation
- `docs`: Contain detail documents
- `line_follower_ws`: catkin working space 
    + `build.bash`: source this file every time import new packages.
    + `devel`: this folder contain built results.
    + `follow_line.py`: main file of the controller and observation logics.
    + `src/line_follower`: main package contains line-world and a ensemble launch file for both world and robot spawner.
    + `src/quad_wheel_robot_6`: package contains config, geometry, controllers, ... related to 4 wheel robot simulation.

