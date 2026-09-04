# Tutorial 5 Notes

**Tutorial 5**: https://docs.google.com/presentation/d/1NSLurHQVMVvQxcS9Ak_V12aZkTbiSgpLeReDgo2snoU/edit?slide=id.p#slide=id.p

## SLAM Toolbox

The **slam_toolbox** package incorporates information from **laser scanners** and **TF transforms** to create a **2D map** of a space.

This package provides a sped-up and improved version of **SLAM Karto**, with an updated SDK and improved visualization and modification toolsets.

https://wiki.ros.org/slam_toolbox

## Before Running slam_toolbox

Make sure the **odometry on your car is properly tuned** before running `slam_toolbox`.

### Installing slam_toolbox

Install the package using:

```bash
sudo apt install ros-foxy-slam-toolbox
```

> **Note:** The command above uses **ROS Foxy**. We use **Humble/Jazzy**, so the ROS distribution in the package name should be changed accordingly.

### Launching slam_toolbox

First, launch **teleop** in one terminal window.

Then launch **slam_toolbox**:

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=path/to/ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
```

### Example

https://www.youtube.com/watch?v=RanGbHii2m8

### Visualization

To visualize the SLAM process:

1. Launch **RViz2**.
2. Add **/map** by topic.
3. Add **/graph_visualization** by topic.
4. In the top-left corner of RViz2, open **Panels → Add New Panel**.
5. Add the **SlamToolBoxPlugin** panel.

Once you are done mapping, **save the map using the plugin**. You can give it a name in the text box next to **Save Map**.

The map will be saved in whichever directory you ran `slam_toolbox`.

### slam_toolbox Configuration

Change the following settings in **RViz2** to visualize the SLAM process:

* **Global Options → Fixed Frame:** `map`
* **Map → Topic → History Policy:** `Keep All`
* **Map → Topic → Durability Policy:** `Transient Local`
* **Map → Updated Topic → Durability Policy:** `Transient Local`

## Running the Particle Filter

### range_libc

The **range_libc** library provides different algorithm implementations for **2D raycasting** on **2D occupancy grids**.

https://github.com/f1tenth/range_libc/tree/humble-devel

### Installing range_libc

Clone the repository:

```bash
cd path/to/ws/src
git clone https://github.com/f1tenth/range_libc.git -b humble-devel
```

Install the Python wrapper:

```bash
cd range_libc/pywrapper
WITH_CUDA=ON python setup.py install --user
```

### Installing the Particle Filter

#### Clone the Package

Navigate to your workspace's `src` directory:

```bash
cd path/to/ws/src
```

Clone the particle filter package:

```bash
git clone https://github.com/f1tenth/particle_filter.git -b humble-devel
```

#### Install Dependencies

Navigate to your workspace:

```bash
cd path/to/ws
```

Install the required dependencies:

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

#### Compile the Workspace

Build the workspace again:

```bash
cd path/to/ws
colcon build
```

Then source the workspace:

```bash
source install/setup.bash
```

### Running the Particle Filter

First, launch **teleop**.

Then launch the particle filter:

```bash
ros2 launch particle_filter localize_launch.py
```

### Visualization

To visualize the particle filter:

1. Run `rviz2`.
2. Use **Add → By Topic** to add **/map**.
3. In the settings for the map, change the **Durability Policy** under **Topic** to **Transient Local**.
4. To show the current localization, add **/pf/viz/inferred_pose**.
5. Optionally, add **/pf/viz/particles** to visualize the individual particles.

### Checking the Update Frequency

Check the publishing frequency of:

```text
/pf/viz/inferred_pose
```

It should be **at least 30 Hz**.

### Changing the Map Used

Place the **map files** (image + `.yaml`) in:

```text
particle_filter/maps
```

Then change:

```text
particle_filter/config/localize.yaml
```

to reflect the **map you want to use**.

### Setting the Initial Position

Use **2D Pose Estimate** in RViz2 to set the **initial position and orientation** of the car on the map.
