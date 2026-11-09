# Navigation

This section explains how to create a map of the environment, configure localization using a previously saved map, and let the robot automatically follow a predefined path.

The navigation system consists of three main steps:

1. **Mapping:** Create a map of the environment using SLAM and record the robot's initial position.
2. **Localization:** Load the saved map and initialize AMCL with the previously recorded starting position.
3. **Path Following:** Start the navigation stack to let the robot follow a predefined path.

Make sure that the robot is properly started and that all required sensors (especially the LiDAR) are publishing data before starting any of these steps.

## How to create a Map

To navigate autonomously, the robot first needs a map of its environment. This map can be generated using SLAM (Simultaneous Localization and Mapping).

### 1. Launch SLAM

Start the SLAM node:

```bash
ros2 launch edu_fleet slam.launch.py
```

Once SLAM is running, move the robot through the environment you want to map. Make sure to cover the entire area and avoid moving too quickly, as this can negatively affect the quality of the resulting map.

You can monitor the mapping process using RViz to check whether the map is being generated correctly.

### 2. Save the Map

Once you have finished mapping the environment, save the generated map to your filesystem.

First, navigate to the folder where you want to save the map. Then execute:

```bash
ros2 run nav2_map_server map_saver_cli -f ./map-name --ros-args -r map:=/eduard/map
```

Replace `map-name` with the desired name of your map.

**Important:** The map topic used for remapping (`/eduard/map`) must match the actual map topic published by your SLAM node.

To find the correct topic name, open a new terminal and execute:

```bash
ros2 topic list
```

Look for the topic carrying the occupancy grid map. It is usually named `/map`, but may include a namespace, such as `/eduard/map`.

You can verify that you found the correct topic by checking its message type:

```bash
ros2 topic info /eduard/map
```

The expected message type is `nav_msgs/msg/OccupancyGrid`.

If your map topic has a different name, adjust the remapping in the map-saving command accordingly.

**Result:** The map saver generates two files:

- `map-name.yaml`: Contains map metadata, such as resolution, origin, and the image filename.
- `map-name.pgm`: Contains the actual occupancy grid image.

Keep both files together, as they are required to load the map later.

### 3. Determine the initial robot position

**Important: The robot must always start at the same physical position and orientation when using the saved map with AMCL.**

To ensure that AMCL can initialize correctly, we need to determine the robot's starting position relative to the map.

1. Place the robot at the exact position and orientation where it will always start when using localization.
2. Make sure SLAM is still running and publishing the map-to-robot TF transform.
3. Open a new terminal and execute:

```bash
ros2 run tf2_ros tf2_echo eduard/map eduard/base_footprint
```

This command continuously displays the transformation between the map frame and the robot's base frame.

You might initially see a message indicating that the transform is unavailable. Wait until SLAM has established the transform.

The output should look similar to this:

```text
At time 4973.136000000
- Translation: [0.460, -1.586, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.015, 1.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.030]
- Rotation: in RPY (degree) [0.000, -0.000, 1.722]
```

Press `Ctrl+C` once you have recorded the values.

**How to interpret the output:**

You only need four values for the initial pose configuration:

| TF2 output | AMCL parameter | Description |
|---|---|---|
| Translation X | `x` | Robot position along the map's X-axis |
| Translation Y | `y` | Robot position along the map's Y-axis |
| Translation Z | `z` | Robot height (usually 0 for a ground robot) |
| Rotation RPY (radian), Z | `yaw` | Robot orientation around the vertical axis |

**Important:** Use the yaw value in radians, not degrees. It is the third value in the RPY (radian) output.

For the example above, the initial pose is:

```yaml
initial_pose:
  x: 0.460
  y: -1.586
  z: 0.0
  yaw: 0.030
```

Save these values. You will need them when configuring AMCL.

#### Finding the correct TF frame names

The example command uses these frames:

- `eduard/map`: The global map frame.
- `eduard/base_footprint`: The robot's base frame.

Your setup might use different frame names, especially if you are working with a different robot namespace.

To find the available TF frames, execute:

```bash
ros2 run tf2_tools view_frames
```

This generates a PDF containing the current TF tree. Open `frames.pdf` and look for:

- The global map frame (usually `map`).
- The robot base frame (usually `base_link` or `base_footprint`).

Use the actual frame names from your TF tree in the `tf2_echo` command.

For example, if your frames are simply `map` and `base_link`, use:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

The first argument is the target frame (map), and the second argument is the source frame (robot base).

## How to run localization with a saved map (default mode)

**This requires a previously saved map and a recorded initial robot position.**

Once a map has been created, the robot can use it to determine its position within the environment. This process is called localization.

The robot uses AMCL (Adaptive Monte Carlo Localization) to estimate its position based on LiDAR measurements and the previously saved map.

### 1. Configure the map

Open the [AMCL launch file](../../launch/navigation/amcl.launch.py).

At the top of the file, locate the `map_file` parameter and change it to the path of the map you want to load.

Make sure that the path points to the `.yaml` file generated when saving the map. The corresponding `.pgm` file must also be present in the same directory.

### 2. Configure the initial pose

Open the AMCL parameter file:

`../../parameter/navigation/nav2_amcl.yaml`

Find the following block:

```yaml
initial_pose:
  x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0
```

Replace the values with the ones you recorded during the mapping process.

**Important:**

- These values are specific to the starting position and the map you are using.
- The robot must physically be placed at this position and orientation every time localization is started.
- If you change the map or the robot's starting position, you must determine and configure the initial pose again.

### 3. Start localization

After configuring the map and initial pose, launch AMCL:

```bash
ros2 launch edu_fleet amcl.launch.py
```

AMCL will load the specified map and use the configured initial pose to initialize its localization.

Once AMCL is running, open RViz and verify that:

- The map is displayed correctly.
- The LiDAR scan aligns with the walls and obstacles in the map.
- The robot's estimated position matches its actual position.
- The estimated position follows the robot's movement.

Once localization is stable, you can start autonomous path following.

## How to run automatic path following

**This requires localization to be running successfully.**

The path-following functionality allows the robot to autonomously follow a predefined path.

Before starting, make sure that:

- The robot is running and publishing odometry.
- The LiDAR and other required sensors are publishing data.
- AMCL is running and the robot is correctly localized.
- The desired path or waypoints are configured in the corresponding configuration files.

### Start path following

Launch the Nav2 path-following stack:

```bash
ros2 launch edu_fleet nav2-path-following.launch.py
```

This starts the navigation components required for autonomous path following.

Depending on the configuration, the robot will start following the predefined path once the navigation stack has initialized and received its goal.

You can use RViz to monitor the robot's position, path, and navigation status.

**Troubleshooting:**

If the robot does not start moving, check the following:

- **Localization:** Is AMCL running, and does the robot have a valid position estimate?
- **TF transforms:** Are the required coordinate transforms (`map`, `odom`, and the robot base frame) available?
- **Path configuration:** Is the correct path or set of waypoints loaded?
- **Topics:** Are odometry, LiDAR, and velocity command topics being published and received?
- **Navigation status:** Check the terminal output for errors during Nav2 initialization.

For debugging, you can inspect active topics using `ros2 topic list` and monitor individual topics using `ros2 topic echo`.