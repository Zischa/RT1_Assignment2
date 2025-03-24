How to Run
==========

This page provides step-by-step instructions for setting up and running the project.

Requirements
------------
Make sure the following prerequisites are met before running the project:
  - A ROS environment (Noetic) is installed on your system.
  - The ROS workspace is built and sourced.
  - `assignment_2_2024 package <https://github.com/CarmineD8/assignment_2_2024>`_
  - xterm emulator for managing multiple terminals

Steps to Instal xterm
---------------------
Open the command shell and run:

.. code-block:: bash

   sudo apt update
   sudo apt install xterm

Steps to Run the Project
------------------------

1. **Copy the git repository:**

.. code-block:: bash

   git clone https://github.com/CarmineD8/rt2_assignment1.git


2. **Build the Workspace:**

.. code-block:: bash

   cd ~/catkin_ws
   catkin_make

3. **Source the Workspace**

.. code-block:: bash
   
   source ~/catkin_ws/devel/setup.bash

4. **Run the package**
Since it's needed to run two different packages, it's been created a launch file to make it more easier.

.. code-block:: bash
   
   roslaunch assignment_2_2024_client assignment2.launch
   
5. **Check last target position** 
If you want to check the last target's position, for which the service node is been build:

.. code-block:: bash
   
   rosservice call /target_srv
