# The warehouse map

`warehouse.yaml` and `warehouse.pgm` in this folder are the map AMCL localizes on:
`sim.launch.py localization:=amcl` serves this map unless `map:=` names another one. The map is
made with slam_toolbox on the scripted survey route, checked against the layout with
`check_map`, and committed (plan F5, P2-D15: 0.05 m cells, trinary PGM).

**Status: not made yet.** The map is made and checked in a Gazebo run, and this README then
records how it was made (below). Until then, `localization:=amcl` needs a map given with `map:=`.
Draw one from the layout, as an ideal SLAM run would see it (plan risk R6):

```bash
ros2 run warehouse_layout rasterize --out /tmp/layout_map
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=amcl map:=/tmp/layout_map.yaml
```

## How to make the map

From the workspace root, in four terminals:

```bash
# 1. The warehouse with slam_toolbox mapping (RViz shows the map growing)
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=slam

# 2. Drive the survey route from the layout (or drive by teleop). It starts at robot1's spawn
#    pose, the map's origin, so the map lines up with the world (plan P2-D2).
ros2 run warehouse_system_tests drive_route --route survey --speed 0.4

# 3. When the route is done: save the map from /map (trinary, 0.65 / 0.25 thresholds)
ros2 run nav2_map_server map_saver_cli -f src/warehouse_navigation/maps/warehouse -t /map \
  --fmt pgm --ros-args -p use_sim_time:=true

# 4. Check it against the layout; look at the overlay too
ros2 run warehouse_layout check_map src/warehouse_navigation/maps/warehouse.yaml \
  --overlay /tmp/overlay.png
```

Then build `warehouse_navigation` again (`colcon build --symlink-install --packages-select
warehouse_navigation`). A symlink install links the files that exist when it is built, so a map
saved later reaches the install only after the next build.

## The map in this folder

Filled in when the map is saved:

| | |
|---|---|
| Made | (date, branch and commit) |
| Command | (the route and speed, or teleop) |
| Layout | (the layout.yaml it was made from) |
| `check_map` | (offset, rotation, distance percentiles, stray cells) |
