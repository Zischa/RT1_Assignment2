
---

# Assignment 2 - 2024: Client-Service Robot Control

This assignment involves creating a ROS package that allows controlling a robot through a client-service interaction. The system should enable the user to set a target for the robot, and the robot should attempt to reach that target. Additionally, a service node will provide the last position of the last target set by the user.

## Prerequisites

The following prerequisites are required to run this package:

- **ROS environment** (ROS Noetic or similar)
- **assignment_2_2024** package
- **xterm** emulator for managing multiple terminals

To install **xterm** on your system, follow the steps below:

### Install xterm

For Debian-based systems (e.g., Ubuntu), run:

```bash
sudo apt update
sudo apt install xterm
```

### Clone the `assignment_2_2024` package

Clone the package repository to your workspace:

```bash
git clone https://github.com/CarmineD8/rt2_assignment1.git
```

## Components of the `assignment_2_2024_client` package

The following programs have been developed for this project:

- **target_client.py**: Assigns a target to the robot and allows it to be cleared.
- **service_node.py**: Provides the last position of the last target set by the user.
- **assignment2.launch**: A launch file that runs all the programs simultaneously with a single command.

## Running the Programs

Once **xterm** is installed, you can run the system by executing the following command:

```bash
roslaunch assignment_2_2024_client assignment2.launch
```

This command will open three separate terminals:

1. The terminal from which you ran the `roslaunch` command will display the process of launching the nodes.
2. The second terminal will allow you to input the target coordinates for the robot.
3. The third terminal will show the status of the service node, indicating that it is running.

---

