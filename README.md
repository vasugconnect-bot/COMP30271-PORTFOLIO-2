# NTU_RobotSim_Nav2
NTU Robot Simulation for Nav2 using ROS 2 Humble and Gazebo Fortress

## Building the Project

From your workspace folder
```bash
colcon build --symlink-install
```

## Launching Simulations

### Launch Maze Simulation
```bash
source install/setup.bash
ros2 launch ntu_robotsim maze.launch.py
```

### Then in a Different Terminal, Launch Single Robot Simulation
```bash
source install/setup.bash
ros2 launch ntu_robotsim single_robot_sim.launch.py
```

## How the Launch Files Work

### 1. Maze Launch File (`maze.launch.py`)

This launch file starts the Gazebo simulation environment:

- **Purpose**: Loads the world/environment in Gazebo Fortress
- **World Loading**: Points to the maze world SDF file located at `worlds/maze/maze.sdf`
- **Gazebo Arguments**: Sets verbosity level to 4 (`-v 4`) for detailed logging
- **Implementation**: Uses `ros_gz_sim` package's `gz_sim.launch.py` to start Gazebo with the specified world

**Key Flow**:
1. Locates the package share directory
2. Constructs path to the maze SDF file
3. Passes the world file path to Gazebo's launch file
4. Gazebo loads the world environment (walls, obstacles, etc.)

### 2. Single Robot Simulation Launch File (`single_robot_sim.launch.py`)

This launch file spawns a robot into the already-running Gazebo world and sets up communication bridges:

#### Configuration Loading
- Reads parameters from `config/single_robot_maze_sim.yaml` or similar world-specific config files
- Configuration includes robot name, SDF file path, spawn position (x, y, z), and orientation (roll, pitch, yaw)

#### Main Components

**a) Robot Spawning** (`spawn_robot.launch.py`)
- Includes the `spawn_robot.launch.py` file with parameters from the config
- Parameters are converted from booleans to strings for compatibility
- The spawn script handles the actual SDF loading process

**b) ROS-Gazebo Bridge** (`ros_gz_bridge`)
- Establishes communication between ROS 2 topics and Gazebo topics
- Uses configuration from `config/single_robot_ros_gz_bridge.yaml`
- If a namespace is specified, creates a namespaced version of the bridge config
- Bridges topics like cmd_vel, sensor data, etc.

**c) Robot State Publisher**
- Publishes static transforms from the URDF file (`urdf/jetbot.urdf`)
- Maintains the robot's TF tree for ROS 2
- Uses simulation time (`use_sim_time: True`)

### 3. Spawn Robot Launch File (`spawn_robot.launch.py`)

This is the core file that handles SDF loading and robot spawning:

#### How SDF Files are Loaded

**Step 1: Configuration Loading**
- Reads `config/spawn_robot.yaml` for default parameters
- Command-line arguments can override these defaults
- Parameters include: robot_name, sdf_file, world name, pose (x, y, z, roll, pitch, yaw), use_imu

**Step 2: SDF File Location**
- Locates the SDF file in the package's models directory
- Default path: `models/jetbot/model.sdf`
- Full path construction: `{package_share_dir}/models/{sdf_file}`

**Step 3: SDF Namespacing (if robot_name is provided)**
- Creates a copy of the original SDF file with namespace suffix
- Example: `model.sdf` → `model_atlas.sdf` (if robot_name is "atlas")
- Modifies the SDF to namespace topics, frames, and sensor names
- This allows multiple robots without topic name conflicts
- Note: Namespace modification code is commented out in the current version

**Step 4: Pose Calculation**
- Converts Euler angles (roll, pitch, yaw) to quaternions using the `get_quaternion_from_euler()` function
- Prepares position (x, y, z) and orientation (qx, qy, qz, qw) for spawning

**Step 5: Gazebo Service Call**
- Uses Ignition Gazebo's `/world/{world_name}/create` service
- Service type: `ignition.msgs.EntityFactory`
- Sends the SDF file path, robot name, and pose to Gazebo
- Gazebo loads the SDF and instantiates the robot model at the specified location

**Example Service Call**:
```
ign service -s /world/maze/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --req 'sdf_filename: "/path/to/model.sdf", name: "atlas", pose: {position: {x: -7.0, y: 0.0, z: 3.96}, orientation: {x: 0, y: 0, z: 0, w: 1}}'
```

### Launch Sequence Summary

1. **First Terminal**: `maze.launch.py` loads the Gazebo world (environment only)
2. **Second Terminal**: `single_robot_sim.launch.py` does three things:
   - Spawns the robot by loading its SDF file into the running world
   - Starts the ROS-Gazebo bridge for topic communication
   - Starts the robot state publisher for TF transforms

### SDF File Structure

The robot SDF files (e.g., `models/jetbot/model.sdf`) contain:
- Visual and collision geometries (meshes, shapes)
- Physical properties (mass, inertia)
- Sensor definitions (cameras, LiDAR, IMU)
- Plugin configurations (differential drive, sensors)
- Joint definitions and relationships

When loaded, Gazebo parses this SDF and creates the complete robot simulation including physics, sensors, and control interfaces.

### URDF File and Its Role

The URDF (Unified Robot Description Format) file (`urdf/jetbot.urdf`) serves a different but complementary purpose to the SDF file:

#### Purpose of URDF
- **TF Tree Management**: Defines the static transform tree for ROS 2's TF system
- **ROS 2 Integration**: Provides robot structure information to ROS 2 nodes and tools (RViz, navigation stack, etc.)
- **Frame Relationships**: Establishes parent-child relationships between different parts of the robot

#### URDF vs SDF
| Aspect | URDF | SDF |
|--------|------|-----|
| **Primary Use** | ROS 2 TF transforms and robot description | Gazebo simulation (physics, sensors, plugins) |
| **Physics** | No physics properties | Full physics simulation (mass, inertia, friction) |
| **Sensors** | Frame definitions only | Complete sensor simulation with plugins |
| **Visualization** | Basic geometry for RViz | Detailed meshes and materials for Gazebo |
| **Scope** | Kinematic structure | Complete simulation model |

#### Jetbot URDF Structure

The `jetbot.urdf` file defines three links and their relationships:

**Links:**
1. **`base_link`**: The main body of the robot (398mm × 268mm × 160mm box)
   - Acts as the root frame for the robot
   - Reference point for all other frames

2. **`base_laser`**: The LiDAR sensor frame (30mm radius, 40mm height cylinder)
   - Connected to `base_link` via `base_to_laser` fixed joint
   - Offset: 280mm above base_link (z=0.28)
   - Used for laser scan data in ROS 2

3. **`camera_link`**: The camera sensor frame (20mm × 20mm × 20mm box)
   - Connected to `base_link` via `base_to_camera` fixed joint
   - Offset: 100mm forward (x=0.1), 190mm above (z=0.19) base_link
   - Used for camera/RGBD data in ROS 2

**Fixed Joints:**
- All joints are `fixed` type, meaning no movement between frames
- Define static transforms that the robot_state_publisher broadcasts continuously

#### How URDF is Used in the Launch System

1. **Loading**: `single_robot_sim.launch.py` reads the URDF file and extracts the robot description
2. **Publishing**: The `robot_state_publisher` node receives the URDF content as the `robot_description` parameter
3. **TF Broadcasting**: The robot_state_publisher broadcasts all static transforms defined in the URDF
4. **ROS 2 Integration**: Other ROS 2 nodes (navigation, visualization) use these TF frames to understand the robot's structure
5. **Sensor Frame Alignment**: Sensor data from Gazebo (via ros_gz_bridge) is associated with the correct frames (base_laser, camera_link)

#### Why Both URDF and SDF?

The system uses both because:
- **SDF** handles the Gazebo simulation side (physics, sensor simulation, actuators)
- **URDF** handles the ROS 2 side (TF tree, navigation stack, planning)
- The ros_gz_bridge connects the two, mapping sensor data from Gazebo topics to ROS 2 topics with proper frame associations
- This separation allows flexibility: you can change simulation details in SDF without affecting ROS 2 navigation, and vice versa

### Map Files for Nav2

The `map/` directory contains map files used by the Nav2 navigation stack for localization and path planning:

#### Map File Components

**1. `map.pgm` - The Map Image**
- A grayscale image representing the environment
- **Black pixels (0)**: Occupied space (walls, obstacles)
- **White pixels (255)**: Free space (navigable areas)
- **Gray pixels (127)**: Unknown space (unexplored areas)
- Format: Portable Gray Map (PGM) - a simple raster image format

**2. `map.yaml` - Map Metadata**
Contains essential parameters for interpreting the map image:

```yaml
resolution: 0.100000        # meters per pixel (10 cm per pixel)
origin: [-6.0, -6.0, 0.0]   # position of pixel (0,0) in world coordinates [x, y, θ]
occupied_thresh: 0.95       # probability threshold for occupied cells (95%)
free_thresh: 0.90           # probability threshold for free cells (90%)
negate: 0                   # whether to invert black/white interpretation
mode: trinary               # map representation mode (trinary = free/occupied/unknown)
image: map.pgm              # path to the map image file
```

#### Key Parameters Explained

**Resolution (0.1 m/pixel)**
- Each pixel in the map image represents a 10cm × 10cm area in the real world
- Higher resolution (smaller values) = more detail but larger file size and computation
- Lower resolution (larger values) = less detail but faster processing

**Origin [-6.0, -6.0, 0.0]**
- Defines where pixel (0,0) of the image corresponds to in the Gazebo world coordinates
- In this case, the bottom-left corner of the map is at position (-6.0, -6.0) in meters
- The third value (0.0) is the rotation angle in radians
- This aligns the map coordinate frame with the Gazebo world frame

**Occupied Threshold (0.95)**
- Probability above which a cell is considered occupied (obstacle)
- Values 0.95-1.0 are treated as obstacles
- Higher values = more conservative obstacle detection

**Free Threshold (0.90)**
- Probability below which a cell is considered free (navigable)
- Values 0.0-0.90 are treated as free space
- Values between free_thresh and occupied_thresh are unknown

**Mode: Trinary**
- Three states: Free, Occupied, Unknown
- Alternative modes: scale (grayscale probabilities), raw (direct pixel values)

#### How Nav2 Uses the Map

**1. AMCL (Adaptive Monte Carlo Localization)**
- Uses the map to estimate the robot's position
- Compares laser scan data with the known map to localize the robot
- Requires the map to match the actual environment

**2. Global Planner**
- Uses the map to plan collision-free paths from start to goal
- Algorithms like A*, Dijkstra, or Smac Planner navigate around obstacles marked in the map
- Plans in the map's 2D grid space

**3. Local Planner/Controller**
- Uses the map along with real-time sensor data for local obstacle avoidance
- The map provides static obstacle information
- Costmaps combine map data with sensor data for dynamic planning

**4. Costmap 2D**
- Converts the map into a costmap with different cost zones
- **Lethal obstacles**: Cells from occupied map areas
- **Inflation layer**: Adds safety margins around obstacles
- **Static layer**: Uses the map as the base layer

---

## Gazebo Resource Path Hook

The `hooks/gz_sim_resource_path.dsv.in` file is an Ament environment hook that automatically configures Gazebo's resource search paths when you source the workspace.

### Purpose

This hook file tells Gazebo Fortress (Ignition Gazebo) where to find custom models and worlds defined in this package without requiring manual environment variable setup.

### What It Does

When you run `source install/setup.bash`, this hook automatically prepends two directories to the `IGN_GAZEBO_RESOURCE_PATH` environment variable:

1. **`share/ntu_robotsim/worlds`** - Allows Gazebo to locate world SDF files (like `maze.sdf`)
2. **`share/ntu_robotsim/models`** - Allows Gazebo to find robot and object models (like `jetbot/model.sdf`)

### Why This Is Important

**Without this hook:**
- You would need to manually set `IGN_GAZEBO_RESOURCE_PATH` every time
- Gazebo wouldn't find your custom worlds and models
- Launch files would fail when trying to load `maze.sdf` or spawn robots

**With this hook:**
- Automatic configuration when sourcing the workspace
- Gazebo can locate all custom resources seamlessly
- Cleaner workflow without manual environment setup

### How It Works

1. During `colcon build`, CMake processes the `.dsv.in` template file
2. The `@PROJECT_NAME@` placeholder is replaced with `ntu_robotsim`
3. The resulting `.dsv` file is installed to `install/ntu_robotsim/share/ntu_robotsim/hooks/`
4. When you source `install/setup.bash`, the hook automatically modifies your environment
5. Gazebo can now find resources in the specified directories

This is the standard ROS 2/Ament way of managing environment variables for packages that need to extend search paths for external tools like Gazebo, RViz, or other simulators.

