# ROS2 Humble Turtle Follower

ROS 2 Humble turtlesim project where one keyboard-controlled turtle leads and a second turtle autonomously follows its path using custom publisher–subscriber nodes and a launch file. Tested on Ubuntu/WSL2 with ROS 2 Humble and turtlesim.

## Demo

[![Turtlesim leader–follower demo](https://img.youtube.com/vi/0o_R2Qvil7Y/0.jpg)](https://www.youtube.com/watch?v=0o_R2Qvil7Y)

## Features

- Keyboard-controlled leader turtle (`turtle1`) using `turtle_teleop_key`.
- Autonomous follower turtle (`turtle2`) that tracks the leader’s pose in real time.
- Custom Python nodes using ROS 2 publishers and subscribers with proportional control.
- ROS 2 Python launch file to start turtlesim and all nodes together.
- Clean ROS 2 Humble package structure with tests and configuration files.

## Prerequisites

- Ubuntu (or WSL2) with **ROS 2 Humble** installed and sourced.
- `turtlesim` package installed:

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

- `colcon` build tools:

```bash
sudo apt install python3-colcon-common-extensions
```

## How to run

Clone the repository:

```bash
git clone https://github.com/USTAT19/ROS2Humble_Turtle_Follower.git
cd ROS2Humble_Turtle_Follower
```
Build the package:

```bash
colcon build --packages-select turtle_follower
source install/setup.bash
```

Launch turtlesim + leader + follower nodes:

```bash
ros2 launch turtle_follower leader_follower.launch.py
```
In a **new terminal** (also sourced):

```bash
ros2 run turtlesim turtle_teleop_key
```
The keyboard teleop node is run separately so it can capture terminal input.

Now `turtle1` is driven by your keyboard, and `turtle2` follows its path automatically.

## Project structure

ROS2Humble_Turtle_Follower/
├── launch/
│ └── leader_follower.launch.py # Starts turtlesim and nodes
├── turtle_follower/
│ ├── __init__.py
│ ├── publisher_.py # Leader pose bridge node
│ ├── follower.py # Follower control node
│ └── turtle_spawner.py # Helper to spawn turtle2 (optional)
├── package.xml
├── setup.cfg
├── setup.py
├── test/
│ ├── test_pep257.py
│ ├── test_flake8.py
│ └── test_copyright.py
└── README.md


## License

This project is licensed under the **MIT License** – see the [LICENSE](LICENSE) file for details.




