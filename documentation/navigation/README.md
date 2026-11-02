# Navigation

## Launch SLAM

```bash
ros2 launch edu_fleet slam.launch.py
```

### Save Map

- Navigate to the folder where you want to save the map
- run
```bash
ros2 run nav2_map_server map_saver_cli -f ./map-name --ros-args -r map:=/eduard/map
```
You might need to change the remapping to match the map topic name


## Localization with AMCL

**this requires a saved map**

- modify the `map_file` parameter at the top of the  [AMCL launch file](../../launch/navigation/amcl.launch.py) to match the map you want to load

- run
```bash
ros2 launch edu_fleet amcl.launch.py
```