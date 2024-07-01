# Explanation for Launch File 

When using ROS, to open world and spawn robot models in Gazebo the `roslaunch` command is used which require a launch file in the package created. In a few steps we will describe some of the key components of the lanch file used for the project.

## World Launch 

In the following snippet an empty world is created in Gazebo and the `course.world` is added with the customizations created.

```xml
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="debug" value="$(arg debug)" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="world_name" value="$(find line_follower)/course.world"/>
  </include>
```

## ROS Nodes 

```xml
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" args="-z 1.0 -unpause -urdf -model robot -param robot_description" respawn="false" output="screen" />

  <node pkg="robot_state_publisher" type="robot_state_publisher"  name="robot_state_publisher">
    <param name="publish_frequency" type="double" value="30.0" />
  </node>
```

## URDF Load 

```xml
  <arg name="model" default="$(find quad_wheel_robot_6)/urdf/quad_wheel_robot_6_motor.urdf"/>

  <arg name="x_pos" default="0.0"/>
  <arg name="y_pos" default="0.0"/>
  <arg name="z_pos" default="0.0"/>
```

## Differential Drive 

```xml
  <node name="rqt_robot_steering" pkg="rqt_robot_steering" type="rqt_robot_steering">
    <param name="default_topic" value="/r2d2_diff_drive_controller/cmd_vel"/>
  </node>
```