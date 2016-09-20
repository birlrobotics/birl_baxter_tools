# Point Cloud Selector

This package will eventually contain programs used to select geometric primitives out of a point cloud using
programs like RVIZ and input devices such as the spacenav.

## Cylinder Selector

### Cylinder Selector Demo

To run the demo do the following

```
rosmake point_cloud_selector
roslaunch point_cloud_selector cylinder_selector_demo.launch
```

This will start RVIZ, 2 GUIs and a fake cylinder selection server. Moving the spacenav will
move the cylinder around in RVIZ. Using the 2 GUIs you can set the cylinder display properties as well
as how the cylinder reponds to spacenav cursor movements.

To see the parameters that represent the cylinder press one of the two buttons on the spacenav.
Pressing the button will print out the pose, height and radius of the cylinder.

### Using the Cylinder Selector in Other Packages

You will need to do 3 things:

1. Implement the select cylinder service server
2. Add the cylinder_cursor to the spacenav_mux as a channel
3. Add a Marker type to RVIZ

To launch the correct nodes with your program simply include the `cylinder_selector.launch` file
in your launch file with the line

```
<include file="$(find point_cloud_selector)/launch/cylinder_selector.launch/>
```

You can also launch the correct nodes by running

```
roslaunch point_cloud_selector cylinder_selector.launch
```

#### Implement the Select Cylinder Service Server

You will need to create a ROS Service Server. You can see an example of this
in the `point_cloud_selector/test/cylinder_properties_server.cpp` file. You can also 
find detailed documentation on implementing a ROS Service Server [here](http://www.ros.org/wiki/roscpp/Overview/Services).

#### Add the cylinder_cursor to the spacenav_mux as a channel

If the spacenav mux is already running you can simply click the add button and enter
`cylinder_cursor` in the text box. You can also add the channel as a default channel.

See the spacenav_mux documentation for more details.

#### Add a Marker to RVIZ

In RVIZ click the add button and select Marker from the list. Set the topic to `point_cloud_selector/cylinder_selector`. The marker will now show up in rviz and can be moved with the spacenav when the correct mux channel is
selected.

### The GUIs

2 GUIs are started. You can control how the cylinder responds to the spacenav with
the cylinder_cursor gui. With the cylinder_selector gui you can control the cylinder's height,
radius, color and transparency. The cylinder marker will update live in RVIZ as you change these values.


