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