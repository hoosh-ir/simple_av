# Simple_AV

## Overview

**Simple_AV** is an autonomous vehicle simulation project designed to integrate various modules, including perception, localization, planning, and control, within a simulated environment provided by AWSIM. The primary purpose of this project is to create a functional and modular autonomous vehicle system that can interact with a simulated environment, enabling testing and development in a controlled setting. Simple_AV offers a simplified alternative to Autoware for autonomous vehicle control. Built with simplicity and flexibility in mind, Simple_AV provides a straightforward approach to vehicle navigation and control, making it an ideal choice for developers seeking a lightweight solution without sacrificing functionality.

For a detailed explanation of the project, please refer to the [documentation](https://hoosh-ir.github.io/simple_av/) hosted on GitHub Pages.

## Key Features:

- **AWSim Integration:** Simple_AV seamlessly connects to the AWSim environment, enabling communication with simulated vehicles and environments. By subscribing to AWSim topics, Simple_AV gathers essential data for navigation and control.

- **Simplified Control Logic:** Simple_AV adopts a minimalist approach to control logic, focusing on essential functionalities for autonomous navigation. By utilizing different approaches, Simple_AV offers flexibility in control strategies, catering to various use cases and requirements.

- **Modular Design:** With a modular architecture, Simple_AV facilitates easy customization and extension. Developers can seamlessly integrate additional functionalities or modify existing modules to suit specific project needs.

- **User-Friendly Configuration:** Simple_AV prioritizes ease of use, providing intuitive configuration options for seamless setup and deployment. Developers can quickly configure parameters to adapt Simple_AV to specific vehicle models or simulation environments.

- **Open-Source and Community-Driven:** Simple_AV is an open-source project, fostering collaboration and innovation within the robotics community. Developers are encouraged to contribute, share insights, and extend the functionality of Simple_AV for collective benefit.

By offering a lightweight alternative to Autoware, Simple_AV empowers developers to streamline autonomous vehicle development workflows, reduce complexity, and accelerate project deployment. Whether used for research, prototyping, or industrial applications, Simple_AV delivers simplicity without compromising on performance or functionality.

## Repository Structure

This repository contains the following key directories:

- **simple_av/**: Contains the main nodes for the Simple_AV system, including `control.py`, `localization.py`, `perception.py` and `planning.py`.
- **V2X_messages/**: Includes the custom ROS message definitions that interface with the AWSIM environment.
- **simple_av_msgs/**: Defines internal message types used for communication between the Simple_AV nodes.
- **.github/workflows/**: Contains the GitHub Actions workflows for Continuous Integration (CI) and deployment of the documentation.
- **docs/**: The source files for the project documentation, written using MkDocs.
- **mkdocs.yml**: The configuration file for MkDocs, which defines the structure and theme of the documentation site.

## Getting Started

To use this project, you'll need to set up a ROS 2 workspace and clone the necessary packages.

### Step 0: Setup the Environment.
Refer to the [simple av system setup page](https://hoosh-ir.github.io/simple_av/Simple-AV/SystemSetup/) in the Simple_AV documentation for detailed instructions.

### Step 1: Create a ROS Workspace

First, create a new ROS workspace:

```bash
mkdir -p ~/ros_ws/src
```

### Step 2: Clone the Repository

Next, clone this repository into the src directory of your workspace:
```bash
cd ~/ros_ws/src
git clone https://github.com/hoosh-ir/simple_av.git
```

### Step 3: Build the Workspace

After cloning the repository, build the workspace using colcon:
```bash
cd ~/ros_ws/
colcon build
```

The workspace should look like this after running `colcon build`:
```bash
ros_ws/
├── build/
├── install/
├── log/
└── src/
    ├── docs/
    ├── mkdocs.yml
    ├── V2X_messages/
    ├── simple_av_msgs/
    └── simple_av/
        ├── package.xml
        ├── resource/
        ├── setup.cfg
        ├── launch/
        │   └── simple_av_launch.py
        └── simple_av/
            ├── control.py
            ├── localization.py
            └── planning.py
```

### Step 4: Source the Workspace

Finally, source the workspace to make the ROS packages available:
```bash
source ~/ros_ws/install/setup.bash
```

### Step 5: Running the project

You can run the project by either running the launch file or by manually starting each node in sequence.

Run the launch file:
```bash
ros2 launch simple_av simple_av_launch.py
```

Or, run the nodes manually:
```bash
ros2 run simple_av localization
ros2 run simple_av perception
ros2 run simple_av planning
ros2 run simple_av control
```

## Documentation 
Full documentation for the project, including architecture details, usage instructions, and API references, can be found [documentation](https://hoosh-ir.github.io/simple_av/).

## Contributing 
Contributions to the Simple_AV project are welcome! Please fork the repository and submit a pull request for any improvements or bug fixes.

