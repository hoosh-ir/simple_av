# Planning Module

The Planning module in simple-AV is responsible for generating safe and feasible trajectories for the autonomous vehicle. This module utilizes data from the Localization and Perception modules to plan a path that ensures the vehicle navigates effectively while adhering to traffic rules and road conditions. The planned trajectory is then executed by the Control module to move the vehicle along the desired path.

## Overview

The Planning module processes information about the vehicle's current position, detected objects, and the environment to generate a trajectory that the vehicle can follow. This involves several steps, including path planning, behavior planning, and motion planning. Each of these steps ensures that the vehicle's trajectory is safe, smooth, and efficient.

## Key Components

- **Localization Data:** Provides the current position and orientation of the vehicle, which is essential for accurate path planning.
- **Perception Data:** Provides information about surrounding objects, road conditions, and traffic signals. This data is critical for avoiding obstacles and adhering to traffic rules.
- **Map Data:** Includes information about the road network, lanelets, traffic lights, and waypoints. This data is used to ensure that the planned path is feasible and adheres to road rules.

## Functionality

The Planning module performs several critical functions to ensure that the vehicle follows a safe and efficient path. The process of path planning consists of three main parts:

### Mission Planning

The mission planning phase determines the path that the vehicle should take from the source point (obtained from the Localization module) to the destination lane (specified in the code). This part uses a Breadth-First Search (BFS) algorithm for routing, covering lane changes if needed. Mission planning only occurs once at the beginning of the process and saves all of the lanes and points from source to destination in a class variable to be used later. Other parts of the Planning module then use this `path` and `path_as_lanes` created by mission planning to perform their tasks.

### Local Planning

The local planning task uses the path of waypoints created in mission planning to determine the look-ahead point based on the defined look-ahead distance. This point is then published to be used in the Control module. 

### Behavioral Planning

Behavioral planning determines the high-level maneuvers the vehicle should perform, such as lane changes, turns, and stops at traffic lights. This step ensures that the vehicle behaves in a manner that is safe and predictable.

## Handling Stops

Since the vehicle needs to stop at certain points along the path, such as at the destination and at traffic light stop lines, the Planning module also handles these stopping points. Similar to the look-ahead distance, a stop distance (which is larger than the look-ahead distance) is defined. The stop distance acts as a look-ahead for points where the vehicle should stop.

## Topic Creation

The Planning module creates a topic named `simple_av/planning/lookahead_point`. The messages in this topic include:

- **Look-Ahead Point:** The next waypoint the vehicle should aim for.
- **Stop Point:** The point where the vehicle should stop.
- **Status:** A string indicating the vehicle's current action, such as:
    - `Cruise`: The vehicle should continue moving.
    - `Decelerate`: The vehicle should slow down.
    - `Turn`: The vehicle should make a turn.
    - `red_stop` or `amber_stop`: The traffic light is red or amber, and the vehicle must slow down and stop at the stop point.
    - `green_cruise`: The traffic light is green, and the vehicle should continue moving.
    - `Park`: The vehicle must stop completely.
- **Speed Limit:** The speed limit for the current segment of the path.

Look-ahead point and stop points are of type `geometry_msgs.msg/Point`.

## Integration

The Planning module is designed to integrate seamlessly with other modules in the simple-AV architecture. It receives input from the Localization and Perception modules and sends its output to the Control module. This integration ensures that the vehicle can navigate effectively in a dynamic environment while maintaining safety and efficiency.

By utilizing data from multiple sources and employing sophisticated planning algorithms, the Planning module plays a crucial role in the autonomous operation of the vehicle, ensuring that it can navigate complex environments and respond to changing conditions in real-time.
