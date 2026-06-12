# waypoint_navigation_plugin

RViz2 tool plugin for placing and publishing waypoints.
ROS2 port (targeting **Humble / Ubuntu 22.04**) of
[KumarRobotics/waypoint_navigation_plugin](https://github.com/KumarRobotics/waypoint_navigation_plugin),
used by the ROS2 port of `kr_autonomous_flight` 
`client_launch` / `map_plan_launch` RViz configs load the
`waypoint_nav_plugin/WaypointNav` tool from this package at runtime
(pluginlib, by class name — snake declares no build dependency on it).

## Usage

Open RViz2 with the plugin built and sourced in the workspace:

```
rviz2
```

Add the **WaypointNav** Tool from waypoint_navigation_plugin (the "+" button in
the toolbar), or use one of snake's shipped RViz configs which already include it.

![WP1](doc/wp_doc_001.png "WP1")

Click on the tool to add multiple waypoints and drop onto the RViz scene. The
locations can be updated by dragging the interactive marker or by using the
RViz panel.

![WP2](doc/wp_doc_002.png "WP2")

Clicking "Publish Waypoints" on the panel publishes `nav_msgs/msg/Path` on the
topic entered under "Topic".

![WP3](doc/wp_doc_003.png "WP3")

## ROS2 port notes

- **Build system:** catkin → `ament_cmake` (package format 3, CMake 3.16+,
  C++17). Dependencies are now `rclcpp`, `rviz_common`, `rviz_rendering`,
  `interactive_markers`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`,
  `pluginlib`, `tf2`.
- **Plugin registration:** library path in
  `waypoint_nav_plugin_description.xml` changed from
  `lib/libwaypoint_nav_plugin` to `waypoint_nav_plugin` (ROS2 convention) and
  the base class is `rviz_common::Tool` (was `rviz::Tool`).
- **API port:** `rviz::*` → `rviz_common` / `rviz_rendering` (mesh loading,
  tool `save`/`load` config), ROS1 node handle / publisher →
  `rclcpp` node + `create_publisher`, interactive marker server →
  the ROS2 `interactive_markers` API, `ROS_ERROR` → `RCLCPP_*` logging.
- **Feature change — rosbag mission files removed.** The ROS1 version could
  save/load waypoint missions as `.bag` files via the `rosbag` C++ API, which
  has no drop-in ROS2 equivalent. `saveToBag`/`loadFromBag` were removed; the
  save/load dialogs now accept **`.yaml` / `.json`** mission files only.
- **Not ported:** the `launch/` directory still contains the ROS1 XML launch
  (`rviz.launch`) and RViz1 configs — the old
  `roslaunch waypoint_navigation_plugin rviz.launch` test flow does not work
  under ROS2. Launch `rviz2` directly (above) or use snake's RViz configs.
