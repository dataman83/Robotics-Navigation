Autonomous Navigation with ROS & PAT: An advanced robotic navigation system integrating the Robot Operating System (ROS) and the Process Analysis Toolkit (PAT).

Overview:

Leverages the A* pathfinding algorithm for efficient navigation in complex environments.

Utilizes ROS for real-time environment interaction, sensor data processing, and robot control.

Incorporates PAT for detailed process analysis, ensuring continuous system improvement.

Features:

Sensor Reading & Environment Interaction: Captures real-time data from the environment to update the system's perception.

State Generation & Map Creation: Processes sensor data to generate a robot's state and an abstract map of the environment.

PAT Interaction & CSP# Model Generation: Uses PAT to create a formal model of the system's state.

Plan Translation & ROS Execution: Transforms the CSP# Model data into executable commands for the robot via ROS.


Implementation:

ROS gathers grid data, emulating a robot with a 360-degree field of view.

PAT code is generated combining ROS parameters and a preset skeleton.

The output of the PAT code is saved and read by the ROS instance for execution.


Applications:

Ideal for sectors like logistics, healthcare, and manufacturing, enhancing efficiency in tasks such as packaging, product transport, and medication delivery.


Authors:

Chris Wuersching: christopher.wuersching@griffithuni.edu.au

Jaidyn Lewis: jaidyn.lewis@griffithuni.edu.au

Md Saminur Rahman: mdsaminur.rahman@griffithuni.edu.au
