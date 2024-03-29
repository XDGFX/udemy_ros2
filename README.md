# udemy_ros2

Course material from the Udemy ROS2 in Python course.

[Course
Link](https://www.udemy.com/course/ros2-robotics-developer-course-using-ros2-in-python/)


## Creating a new package (Python only)
Create your workspace folder
```bash
mkdir udemy_ros2_ws
```

Create the source folder
```bash 
cd udemy_ros2_ws
mkdir src
```

Create the package folders.
```bash
cd src
ros2 pkg create udemy_ros2_pkg --build-type ament_python
```

## Compiling the package (Python only)
The scripts inside the `project_name` folder can be turned into a Python package by adding an empty `__init__.py` file. In `setup.py` they can then be imported and installed

```python
entry_points={
    'console_scripts': [
        "publisher = udemy_ros2_pkg.publisher:main",
        "subscriber = udemy_ros2_pkg.subscriber:main",
    ],
},
```

The package can be built using
```bash
cd ~ros_workspaces/udemy_ros2_ws
colcon build
```

This will create the installation under `install/`.
To open the workspace with this installed package, run:

```bash
source install/setup.bash
```

And then you can check that the nodes are available using

```bash
ros2 pkg executables udemy_ros2_pkg
```

To run the nodes use
```bash
ros2 run package_name script_name
ros2 run udemy_ros2_pkg publisher
```


## Creating a new package (C++ compatible)
Create your workspace folder
```bash
mkdir udemy_ros2_ws
```

Create the source folder
```bash 
cd udemy_ros2_ws
mkdir src
```

Create the package folders. This uses `ament_cmake`, which uses a
`CMakeLists.txt` file, however this could alternatively be done using
`ament_python`, which uses a conventional Python `setup.py` file. [Example Tutorial.](https://automaticaddison.com/how-to-set-up-a-ros2-project-for-python-foxy-fitzroy/)
```bash
cd src
ros2 pkg create udemy_ros2_pkg --build-type ament_cmake
```

## Compiling the package (C++ compatible)
The scripts inside `/scripts` can be turned into a Python package by adding an empty `__init__.py` file. In `CMakeLists.txt` they can then be imported and installed

```cmake
ament_python_install_package(scripts/)

install(PROGRAMS
scripts/publisher.py
scripts/subscriber.py
DESTINATION lib/${PROJECT_NAME}
)
```

The package can be built using
```bash
colcon build
```

This will create the installation under `install/`.
To open the workspace with this installed package, run:

```bash
source install/setup.bash
```

And then you can check that the nodes are available using

```bash
ros2 pkg executables udemy_ros2_pkg
```

To run the nodes use
```bash
ros2 run package_name script_name.py
ros2 run udemy_ros2_pkg publisher.py
```

## Working with topics
Once you have nodes which publish data to topics, it may be helpful to view the
topics outside of a package for debugging.

To list the topics, first source the workspace as has been done previously,
then:
```bash
ros2 topic list
```

This will show a list of possible topics
```
callum@quokka:~/Workspaces/udemy_ros2_ws$ ros2 topic list
/parameter_events
/rosout
/rpm
/speed
```

You can listen in on a specific topic using the `echo` command
```bash
ros2 topic echo /speed
```
(also using `speed` or `/speed` seem to both work, this may be because the echo
is relative to the root, and so there is no need to specify the leading slash)

## Working with parameters
Parameters are a way of sharing data across different nodes, in a way like
global variables which can be used throughout.

While nodes are running, you can see their parameters using:
```bash
ros2 param list
```

The default `use_sim_time` will likely be the only parameter at default.

You can view the state of parameters using:
```bash
ros2 param get /node parameter
ros2 param get /wheel_speed_pub_node use_sim_time
```

For more information use:
```bash
ros2 param describe /node parameter
ros2 param describe /wheel_speed_pub_node use_sim_time
```

Likewise, you can change the value of a parameter using:
```bash
ros2 param set /node parameter value
ros2 param get /wheel_speed_pub_node wheel_radius 0.5
```

To share parameters between nodes, ROS services can be used. One node can
request the value of a parameter from another node.

Services are available by default to get parameters, these can be seen using:
```bash
ros2 service list
```

## Creating a launch script
Currently nodes are being run one at a time, manually. By creating a launch
script, multiple nodes can be started at once. In ROS1, this was done using
`xml` files, however ROS2 switched to using Python files.

They use the format `name.launch.py`, to indicate they are a launch file, and go
inside a `launch/` folder.

Create a file under `launch/`
```bash
touch launch/rm_node.launch.py
```

To load launch files when the package is compiled, it must be added to the
`setup.py` file:

```python
import os
from glob import glob

...

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
],
```

This tells the setup to look inside the `share/{package_name}/launch` folder and
load all files which end with `launch.py`.

Once compiled with
```bash
colcon build
```

and the workspace is sourced, the launch file can be run using
```bash
ros2 launch package_name launch_file.py
ros2 launch udemy_ros2_pkg rm_node.launch.py
```

#### n.b.
Launch files can be used to run external commands also, by importing
`ExecuteProcess` from the `launch.actions` module.

```python
from launch.actions import ExecuteProcess
```

## Bag files
These are used to record robot data, so it can be reviewed and played back at a
later time.

To start recording into a ROS bag:
```bash
ros2 bag record             -a              -o test.bag
                  include all topics        specify the
                  alternatively use         output bag
                    the topic name           filename
```

Stop recording with `Ctrl+C`

To view info about the bag:
```bash
ros2 bag info test.bag/
```

```log
Files:             test.bag_0.db3
Bag size:          19.3 KiB
Storage id:        sqlite3
Duration:          51.0s
Start:             Jun  3 2021 10:48:40.88 (1622713720.88)
End:               Jun  3 2021 10:49:31.88 (1622713771.88)
Messages:          109
Topic information: Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 5 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 0 | Serialization Format: cdr
                   Topic: /rpm | Type: std_msgs/msg/Float32 | Count: 52 | Serialization Format: cdr
                   Topic: /speed | Type: std_msgs/msg/Float32 | Count: 52 | Serialization Format: cdr
```

Play it back using:
```bash
ros2 bag play -l test.bag/
```
*the `-l` will loop the file after it has completed*

## Installing packages
This section will look at including the official `image_common` package, which
can be used to interface with USB cameras.

[ROS1 Documentation](http://wiki.ros.org/image_common)
[ROS2 Source](https://github.com/ros-perception/image_common/tree/ros2)

There are two ways to install this package, as it is an official ROS package.

1. Install via `apt`
```bash
sudo apt install ros-$ROS_DISTRO-image-common
```
*Note that all spaces are replaced with dashes, and the bash variable
`$ROS_DISTRO` will be replaced with `foxy` in this install.*

It will install into the global ROS foxy location, which (if originally
installed using prebuilt binarys with apt), this is under
```
/opt/ros/foxy/share/image_common
```

2. Bundle with our workspace and compile
   We will bundle the `usb_cam` package, by cloning it into our `/src` folder
   under the workspace
```bash
git clone -b ros2 https://github.com/ros-drivers/usb_cam.git 
```

Alternatively
```bash
git clone https://github.com/ros-drivers/usb_cam.git
cd usb_cam/
git switch ros2
```

## Services
Services and messages can only be used with `CMake` packages! They are not
currently compatible with pure Python packages :(...
To create services, we put them inside the package under the `srv/` folder.

```bash
touch srv/OddEvenCheck.srv
```
*Note that naming convention uses CamelCase.*

The service file contains information about the message it expects to receive,
and the message it should respond with.

Once it's created, we need to add some dependencies to our `package.xml` file so
we can build new interface types.

```xml
<!-- Allow building of service -->
<build_depend>rosidl_default_generators</build_depend>

<!-- Allow running of service -->
<exec_depend>rosidl_default_runtime</exec_depend>

<!-- Add it to the group of default interfaces-->
<member_of_group>rosidl_interface_packages</member_of_group>
```

For more information on what's needed, including what to add to `CMakeLists.txt`,
see (the official documentation.)[https://docs.ros.org/en/ros2_documentation/galactic/Tutorials/Custom-ROS2-Interfaces.html]

## Actions
Actions are similar to services, but are mainly for communication which has
three distinct parts:
- Goal (the initial message)
- Feedback (status updates on the goal)
- Result (final response message)

Actions can go under the `action/` folder, and use a similar naming convention
to services, with the file extension `.action`.

The action file is split into three parts (as shown above), using three dashes,
similar to services.

We need the same dependencies as services in our `package.xml` file:
```xml
<!-- Allow building of service -->
<build_depend>rosidl_default_generators</build_depend>

<!-- Allow running of service -->
<exec_depend>rosidl_default_runtime</exec_depend>

<!-- Add it to the group of default interfaces-->
<member_of_group>rosidl_interface_packages</member_of_group>
```

With an additional dependency:
```xml
<depend>action_msgs</depend>
```

For more information on what's needed, including what to add to `CMakeLists.txt`,
see (the official documentation.)[https://docs.ros.org/en/ros2_documentation/galactic/Tutorials/Custom-ROS2-Interfaces.html]
