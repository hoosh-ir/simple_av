# Localization Module

The Localization module in simple-AV is designed to accurately determine the position and orientation of the autonomous vehicle within its environment using GNSS/pose data. This module is crucial for ensuring that the vehicle can navigate safely and effectively, as it provides the foundational data needed for all subsequent decision-making processes, such as perception, planning, and control.

## Overview

Localization is achieved by utilizing GNSS/pose data to determine the vehicle's position within a predefined map. This map is stored in a JSON file, which includes detailed information about the city's lanelets, their connections, waypoints, and traffic light locations. The Localization module processes this data to identify the vehicle's current lane and the closest waypoint within that lane.

## Key Components

- **GNSS/Pose Data:** Provides the vehicle's current coordinates (x, y, z). This data is used to determine the vehicle's position relative to the predefined map.

- **JSON Map:** Contains all lanelets of the city, including their next, previous, and adjacent lanes. Each lanelet consists of waypoints and traffic light information, providing a comprehensive representation of the driving environment.

## Functionality

The Localization module performs several critical functions to ensure accurate and reliable vehicle positioning:

1. **Global Localization:** At the beginning of the process, the global localization component compares the GNSS/pose data to all the waypoints in each lanelet. This step determines the currently occupied lane and is computationally intensive, so it is only performed initially to establish the vehicle's starting position.

2. **Local Localization:** Once the vehicle's initial position is determined, the local localization component takes over. It creates a search area based on the current lane to limit the computational load. This localized search ensures that the process is efficient and can run in real-time without excessive computational demands.

3. **Lane and Waypoint Determination:** The module continuously checks if the vehicle is within the current lane and identifies which waypoint is closest to the vehicle. This helps in maintaining accurate localization and provides essential data for navigation.

4. **Topic Creation:** The Localization module creates a topic named `simple_av/localization/location`. The messages in this topic include:
    - Current lane
    - Closest waypoint
    - Distance to the closest waypoint

## Integration

The Localization module is tightly integrated with planning components of the simple-AV software stack:

- **Planning Module:** Relies on precise localization to generate safe and feasible trajectories for the vehicle to follow.


## Advantages

- **High Accuracy:** By utilizing GNSS/pose data and a detailed map, the Localization module provides a highly accurate estimate of the vehicle's position and orientation.

- **Efficiency:** The two-tiered approach of global and local localization ensures that the system is both accurate and computationally efficient.

- **Real-Time Operation:** Capable of processing data and updating the vehicle's pose in real-time, essential for dynamic and responsive autonomous driving.

- **Seamless Integration:** Works seamlessly with other modules in the simple-AV stack, facilitating comprehensive and efficient autonomous vehicle operation.

