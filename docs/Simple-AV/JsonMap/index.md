
# Map
Simple-AV uses the simplest form of map. This map which is a Json file, contains some crusial information about the environment which the autonomous vehicle operates in. Below you can see the format designed for this map file. The [Localization module](../Modules/Localization/index.md) uses this format to read the input map file. Also, the [Planning module](../Modules/Path_planning/index.md) uses this map for path planning purposes.

```python
{
    "LaneLetsArray": [
        {
            "name":
            "waypoints": [
                { "x":  "y": "z": },
                { "x":  "y": "z": }
            ],
            "prevLanes": [],
            "nextLanes": [],
            "trafficlightsWayIDs": [],
            "stopLinePoseP1": [],
            "stopLinePoseP2": [],
            "densed_waypoints": [
                { "x":  "y": "z": },
                { "x":  "y": "z": }
            ],
            "adjacentLanes": []
        },
        {
            # data of another lane
        }
    ]
}
```

#### JSON File Layout and Architecture

<b>Root Element:</b>

* <b>LaneLetsArray:</b> An array containing multiple lanelet objects.

Lanelet Object Structure:

* <b>name:</b> The name of the lanelet (string).
* <b>waypoints:</b> An array of waypoint objects, each containing coordinates (x, y, z).
    * <b>x:</b> The x-coordinate of the waypoint (float).
    * <b>y:</b> The y-coordinate of the waypoint (float).
    * <b>z:</b> The z-coordinate of the waypoint (float).
* <b>prevLanes:</b> An array of strings representing the names of previous lanelets.
* <b>nextLanes:</b> An array of strings representing the names of next lanelets.
* <b>adjacentLanes:</b> An array of strings representing the names of adjacent lanelets of the this lane.
* <b>trafficlightsWayIDs:</b> An array of integers representing traffic light way IDs.
* <b>stopLinePoseP1:</b> An array of three floats representing the first point of the stop line pose (x, y, z).
* <b>stopLinePoseP2:</b> An array of three floats representing the second point of the stop line pose (x, y, z).
* <b>densed_waypoints:</b> An array of waypoint objects, which is more densed. The distance between each two consecutive points can be determined. For examle, the distance can be set to 2.0 meters.
    * <b>x:</b> The x-coordinate of the waypoint (float).
    * <b>y:</b> The y-coordinate of the waypoint (float).
    * <b>z:</b> The z-coordinate of the waypoint (float).

## How to generate this map from Awsim.

An important consideration is that this map is generated using AWSIM. AWSIM itself uses PCD and Lanelet map files to create waypoints containing information about lanes. To simplify the process for simple_AV, we eliminate the use of Lanelet and PCD map files, utilizing only the AWSIM waypoint. The JSON file mentioned is created and populated in AWSIM using a C# code. Then we use a python code to update the map by adding the adjacent lanes and densed waypoints to each lane. [Map making repository](https://github.com/hoosh-ir/simple_av_map_maker)