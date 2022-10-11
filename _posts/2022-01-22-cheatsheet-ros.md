---
title: "ROS Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - ros
  - cheatsheet
toc: true
toc_sticky: true
---

# ROS Packages

## image_view

Display the images of an image topic:
```bash
rosrun image_view image_view image:=/sensor/camera1/image_raw
```

## opencv_apps

### Install

- `sudo apt install ros-noetic-opencv-apps`

### Hough Transform

```bash
roslaunch opencv_apps hough_lines.launch image:=/sensor/camera1/image_raw

# view all arguments of a node in the launch file
roslaunch --args hough_lines /opt/ros/noetic/share/opencv_apps/launch/hough_lines.launch

# view all arguments of the launch file
roslaunch --ros-args /opt/ros/noetic/share/opencv_apps/launch/hough_lines.launch
```

# Fast File Search

E.g. to quickly locate and show launch files, `package.xml`, etc.
```bash
rospack find rac<<< hit tab —> racing >>>
roscat <package> <file>
```

# Roscore

To start a **ROS Master** (and some other pre-requisite programs):
```bash
roscore
```

From [ROS wiki](http://wiki.ros.org/roscore):

`roscore` is a collection of nodes and programs that are pre-requisites of a ROS-based system. You **must** have a roscore running in order for ROS nodes to communicate. It is launched using the roscore command.

NOTE: If you use `roslaunch`, it will automatically start `roscore` if it detects that it is not already running (unless the `--wait` argument is supplied).

`roscore` will start up:
- a **ROS Master**
- a ROS Parameter Server
- a rosout logging node 

There are currently no plans to add to `roscore`.

# Roslaunch

Tool for 
- **launching multiple ROS nodes** and 
- **setting parameters on the Parameter Server**.
Takes in one or more **XML configuration files** (with `.launch` extension) that specify 
- the **parameters** to set and 
- **nodes** to launch.

```bash
roslaunch <package> <.launch-file> <<< hit tab 2x >>>		# mit tab 2x werden alle arguments des .launch file angezeigt
```

## Launch Files

`.launch` file tags:
- `node` (run node)
- `param` (set param)
    - if `<param name=… value=… />` is between `<node name=… pkg=… type=… output=… >` and `</node>`, then the final name of the parameter is `node name`/`param name`
- `arg` (take in arguments from outside this file, and otherwise set default)
    - e.g. `<arg name="world_path" default="$(find racing)/resources/racing_world.yaml"/>`

`roslaunch --ros-args /path/to/launchfile.launch`
- Display command-line arguments for this launch file

```xml
<launch>
  <!-- ros_args.launch -->
  <arg name="foo" default="true" doc="I pity the foo'."/>
  <arg name="bar" doc="Someone walks into this."/>
  <arg name="baz" default="false"/>
  <arg name="nop"/>
  <arg name="fix" value="true"/>
</launch>
```

### Show Launch File Information

```bash
$> roslaunch --ros-args ros_args.launch
Required Arguments:
  bar: Someone walks into this.
  nop: undocumented
Optional Arguments:
  baz (default "false"): undocumented
  foo (default "true"): I pity the foo'.
```

# Rosrun

```bash
rosrun <package> <node>
```

# Nodes

```bash
rosnode list
rosnode info <node>
```

# Parameters

```bash
rosparam <<< hit tab 2x >>>
rosparam list
rosparam set <parameter> <value>
rosparam get <parameter>
```

# Messages

```bash
rosmsg show <<< hit tab 2x >>>
rosmsg show <message type>    # (message type: zB geometry_msgs/Twist)
```
alternativ: `rosmsg info`

# Topics

```bash
rostopic list
rostopic info /topic <<< hit tab 2x >>>    # zeigt .msg type des topics (gibt nur einen) UND Liste aller nodes, die zu "/topic" publishen oder "/topic" subscriben
rostopic echo /topic
rostopic pub /topic <message type> <message>
```

# Services

- are similar to publish/subscribe message system, but are **intended for one-time execution** instead of permanent streaming.
- Client **directly** calls service at another node **without going through anonymous topic**. 
- Services natively include **responses** ( publish/subscribe system does not! )
- similar to .msg types, services work with **.srv types** (zB. `flatland_msgs/MoveModel` cp. output of `rosservice info`)

```bash
rosservice list
rosservice info /service
rossrv info <service type>
rosservice call /service <<< hit tab >>>
```

# Bags

(= a bag is a **file format** for storing message data.)

```bash
rosbag record /Topic1 [ /Topic2 /Topic3 … ]			stores recorded .bag file with the contents of the specified topics in the current dir (i.e. use e.g. “roscd racing” before !)
```

**NOTE**: Füge `*.bag` zum `.gitignore` file des branch hinzu ! (Weil `.bag` files werden in real-world projects zu groß (multiple GB) um sie in einer git repo zu tracken !) Dadurch bleibt `git status` "clean", auch wenn ein `.bag` file dazukommt.

```bash
rosbag info bagfile1 [ bagfile2 bagfile3 … ]
```

```bash
rosbag play -l bagfile1 [ bagfile2 bagfile3 … ]    # -l for loop, ie. repeat after "duration" is over (sonst hört ROS nach ca. "duration" auf) (while running, press <space> to pause and resume the playback)
```

# rclcpp (ROS Client Library for C++)

## RWTH Self-Driving Lab 1

### yaml files

`racing_cart.yaml` 
- **Vehicle model** = (**kinematic single track model** [s. Ex5] + **actuator model** [incl. actuators' "lagging"])

### header files

```
VehicleController.h
ros/ros.h
sensor_msgs/LaserScan.h
geometry_msgs/Twist.h
```

### classes

```cpp
ros::Publisher
::publish(const boost::shared_ptr<M>& message)          // overloaded function   // "M" ist ein template parameter (s. C++ templates), d.h. "M" steht für einen beliebigen Typ bzw. Klasse
::publish(const M& message)                                            // overloaded function.  // "M" ist ein template parameter (s. C++ templates), d.h. "M" steht für einen beliebigen Typ bzw. Klasse
ros::Subscriber
ros::NodeHandle
::getParam("vehicle/sensor_topic", subscribe_topic_sensors)
::subscribe(subscribe_topic_sensors, 10, callbackLaserSensor)
::advertise<geometry_msgs::Twist>(publish_topic_actuators, 10)
ros::Rate
```

VehicleController
```cpp
::overwriteLidarDistances(const float distances[3])
::computeTargetValues()
::getTargetVelocity()
::getTargetSteeringAngle()
```

### messages

**Generell gilt**: Schaue message Definitionen in entsprechenden `.msg` files in `noetic/share/sensor_msgs/msg/` nach ! Oder über `rosmsg show` Befehl !

```cpp
geometry_msgs::Twist
geometry_msgs::Vector3
```

### Understanding `sensor_msgs::LaserScanPtr`

- `LaserScanPtr` ist ein `typedef`-Synonym für `boost::shared_ptr< ::sensor_msgs::LaserScan >` 
    - dh. `::sensor_msgs::LaserScan` wird hier als class im **Template Parameter** aufgenommen
        - s. Notes "C++" unter "**Double Colon prepended to the class name**"
- `ContainerAllocator` ist ein `struct`-**Template Parameter** (`struct` templates sind ähnlich wie `class` templates)! (Zeile 22, `LaserScan.h`) und in Zeile 94 wird `std::allocator<void>` als **Template Parameter** übergeben, um den `LaserScan`-Typ über `typedef` zu definieren.
    - unterscheide `LaserScan_` (mit Unterstrich) und `LaserScan` (ohne Unterstrich)
    - `LaserScan` (ohne Unterstrich) ist ein `typedef` für ein `LaserScan_` (mit Unterstrich) -Objekt (Zeile 94 in `LaserScan.h`)
    - `LaserScanPtr` wiederum ist ein `typedef` für ein `LaserScan`-Objekt (Zeile 96) 

#### `::ranges[i]`

- ranges hat den Typ `_ranges_type` (s. `LaserScan.h`), was ein `typedef` ist für `std::vector<float, custom Allocator>`, wobei der custom Allocator über den Template Parameter `ContainerAllocator` (s. Zeile 22 in `LaserScan.h`) festgelegt wird (in Zeile 94 ist der custom Allocator also `std::allocator<void>`
- `std::vector` ist definiert als `template < class T, class Alloc = allocator<T> > class vector;` (s. `<vector>` header file) [tutorialspoint](https://www.tutorialspoint.com/cpp_standard_library/vector.htm)
- [Wikipedia: Allocator](https://en.wikipedia.org/wiki/Allocator_(C%2B%2B))
- zu `template rebind`: [stackoverflow](https://stackoverflow.com/questions/14148756/what-does-template-rebind-do)
- **einfach gesagt**: ranges ist ein beliebig langer `std::vector` mit `float32` Elementen (vgl. `rosmsg show sensor_msgs/LaserScan` Befehl)

### methods

#### ROS_INFO()

```cpp
ROS_INFO("string")
```

#### ros::init()

```cpp
ros::init(argc, argv, "vehicle_controller")
```

#### ros::spin()

```cpp
ros::spin()
```

alternativ zu `ros::spin()`:
```cpp
  ros::Rate r(10); // 10 hz
  while (ros::ok())
  {
    // ... do some work, publish some messages, etc. ...
    ros::spinOnce();
    r.sleep();			// r ist ein ros::Rate Objekt (s. 1. Zeile)
  }
```
