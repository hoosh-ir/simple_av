# Perception Module

The Perception Module in simple-AV is responsible for interpreting data from various sensors to understand the vehicle's surroundings. This module processes information from sensors to detect and identify objects, obstacles, and road features, providing critical inputs for navigation and decision-making.

## Key Functions

1. **Object Detection**:
    - The module identifies and classifies objects in the environment, such as other vehicles, pedestrians, traffic signs, and lane markings.
    - It utilizes data from sensors like LiDARs and cameras to detect objects and their spatial properties.

2. **Traffic Light Detection**:
    - The module detects traffic lights and their statuses (e.g., red, green, amber) to manage the vehicle’s response to traffic signals.
    - It processes visual data from cameras to determine the state of traffic lights at intersections.

3. **Sensor Data Integration**:
    - The Perception Module integrates data from multiple topics, such as v2x and v2i, to create a comprehensive representation of the environment.

