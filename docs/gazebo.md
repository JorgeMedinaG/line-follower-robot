# Explanation for Gazebo World 

For the current project, a custom modification of environment in Gazebo was needed to build a track with a line the robot will be able to follow. Here the steps to do so are explained. 

## Texture 

According to the [Gazebo documentation](https://classic.gazebosim.org/tutorials?tut=color_model#AboutTextures) the textures are used to convert an image onto a shape. 

For creating the custom line track, we used this approach by creating an image that represented the desired track for the robot to follow: 

<img src="../line_follower_ws/src/line_follower/course.png" alt="track" width="400"/>

Then we created the [course.material](../line_follower_ws/src/line_follower/course.material) file, which references the previous image to convert into a texture: 

```
material course
{
  receive_shadows on
  technique
  {
    pass
    {
      ambient 0.5 0.5 0.5 1.0
      texture_unit
      {
        texture course.png
      }
    }
  }
}
```

## Gazebo World 

A [Gazebo World](https://classic.gazebosim.org/tutorials?tut=build_world) is defined as a collection of robots and objects (such as buildings, tables, and lights), and global parameters including the sky, ambient light, and physics properties. 

A world file [course.world](../line_follower_ws/src/line_follower/course.world) was created following some of the values defined in out-of-the-box world files but the custom masterial previously created is inserted in this code snippet: 

```xml
<visual name="ground_vis">
    <geometry>
    <box>
        <size>10 10 .1</size>
    </box>
    </geometry>
    <material>
    <script>
        <uri>file://course.material</uri>
        <name>course</name>
    </script>
    </material>
</visual>
```

## Launch File 

To integrate the custom world in a lunch file, the following tag needs to be added in the tag that launches the empty world. 

```xml
<arg name="world_name" value="$(find line_follower)/course.world"/>
```