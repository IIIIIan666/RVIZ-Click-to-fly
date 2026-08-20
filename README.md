# click_to_fly

Click-to-fly control for a PX4 drone (SITL) via MAVROS. The node accepts target points published from RViz, sends local position setpoints in OFFBOARD mode, and supports recording and replaying trajectories.

## Features

- Take off to a configurable altitude and fly through multiple RViz clicked points in order
- Automatically request PX4 OFFBOARD mode and vehicle arming through MAVROS services
- Record local odometry as `(x, y, z)` waypoints at a configurable interval
- Load recorded waypoints for basic mission-point replay (WIP)
- Reverse an existing trajectory into a new trajectory file
- Validate the initial local position before publishing takeoff setpoints
- Validate clicked-point and odometry frame IDs before accepting a target
- Stop startup when trajectory initialization/loading fails or required parameters are invalid
- Start the control node together with RViz and `rqt_console` through the launch file

## Recent optimization

This update separates trajectory file operations from the flight-control node and improves startup and runtime safety.

### Trajectory utilities

Trajectory operations are now implemented in `file_utils.cpp` and exposed through `file_utils.h`:

- `getFilePath()` resolves trajectory files inside the package `config/` directory.
- `initTraj()` truncates the selected output file and writes the trajectory header.
- `appendWaypoint()` appends one indexed waypoint and reports open/write failures.
- `loadTraj()` ignores blank lines and comments, validates waypoint rows, and returns failure when no valid waypoint is available.
- `saveTraj()` rewrites a complete waypoint vector and checks the final output-stream state.

Both `click_to_fly` and `reverse_traj` link against the same `file_utils` library, so trajectory parsing and serialization use one consistent implementation.

### WRITE mode

At startup, WRITE mode initializes and truncates `config/<write_traj_file_name>`. After a valid odometry message is received, the current local position is appended every `write_interval` seconds. The waypoint index advances only after a successful write.

### READ mode

> **WIP:** READ mode currently provides only the basic waypoint execution path. It has not yet been validated as a complete return-to-home workflow in PX4 SITL.

READ mode loads `config/<read_traj_file_name>`, converts each loaded `Waypoint` into a mission point, and executes the points in file order. A point is considered reached when its three-dimensional distance from the current position is within `mission_tolerance`. Safe return entry, automatic in-memory reversal, interruption handling, and return completion behavior still need to be implemented and tested.

### Initialization and coordinate handling

- The first received odometry message establishes the initial local position; a nonzero altitude is no longer required.
- The takeoff setpoint is calculated only after the initial position is available.
- Startup stops when no local position is received within `init_wait_timeout`, preventing OFFBOARD operation with an unknown origin.
- RViz clicked points are treated as coordinates in the same local frame as MAVROS odometry. If both messages provide a frame ID and the IDs differ, the point is rejected because this node does not currently perform a TF conversion.
- The clicked target altitude is `initial_z + takeoff_altitude`.

## Repository structure

```text
click_to_fly/
├── include/click_to_fly/
│   └── file_utils.h          # Waypoint type and trajectory file API
├── src/
│   ├── click_to_fly.cpp      # Main OFFBOARD control node
│   ├── file_utils.cpp        # Shared trajectory file implementation
│   └── reverse_traj.cpp      # Standalone trajectory reversal node
├── launch/
│   └── click_to_fly.launch   # Control node + RViz + rqt_console
├── config/
│   ├── traj.yaml             # Default recorded/replayed trajectory
│   └── click_to_fly.rviz     # RViz configuration
├── CMakeLists.txt
└── package.xml
```

## Test environment

| Component | Version |
|-----------|---------|
| ROS | Melodic / Noetic (ROS 1) |
| PX4 | 1.x SITL (jMAVSim) |
| MAVROS | `mavros_posix_sitl.launch` |

## Prerequisites

- Ubuntu 18.04/20.04 with ROS Melodic or Noetic
- PX4 SITL environment with MAVROS
- RViz and `rqt_console`
- A catkin workspace containing this package

The package declares `geometry_msgs`, `mavros_msgs`, `nav_msgs`, `roscpp`, and `roslib` as catkin dependencies and compiles as C++11.

## Compile

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Run

### Terminal 1: PX4 SITL

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch px4 mavros_posix_sitl.launch
```

### Terminal 2: click_to_fly

The supplied launch file starts in WRITE mode using `config/traj.yaml`:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch click_to_fly click_to_fly.launch
```

To run the control node directly with custom private parameters:

```bash
# Record a trajectory
rosrun click_to_fly click_to_fly \
  _mode:=WRITE \
  _write_traj_file_name:=traj.yaml \
  _write_interval:=5.0

# Replay a trajectory
rosrun click_to_fly click_to_fly \
  _mode:=READ \
  _read_traj_file_name:=traj.yaml \
  _mission_tolerance:=0.5
```

When using `rosrun`, start RViz separately if clicked-point input or visualization is required.

### Reverse a trajectory

`reverse_traj` reads the input trajectory, reverses the waypoint order, and writes a new file. Both filenames are resolved inside `config/`.

```bash
rosrun click_to_fly reverse_traj \
  _input_file:=write_traj.yaml \
  _output_file:=read_traj.yaml
```

The default input and output names are `write_traj.yaml` and `read_traj.yaml`.

## Usage

### WRITE mode

1. Start PX4 SITL and MAVROS.
2. Start `click_to_fly` and wait for `FCU connected` and the initial local-position message.
3. Wait for OFFBOARD mode and arming to succeed.
4. In RViz, use **Publish Point** to add one or more targets.
5. The drone processes clicked points in order while its local odometry is periodically recorded.

### READ mode

> **WIP:** The following steps describe the current basic playback behavior, not a completed or flight-validated return mode.

1. Make sure the selected trajectory file contains at least one valid waypoint.
2. Start the node with `mode=READ` and the desired `read_traj_file_name`.
3. After initialization, the loaded points are executed in file order.
4. Use `reverse_traj` first when the same path needs to be followed in reverse.

## Parameters

| Parameter | Code default | Launch value | Description |
|-----------|--------------|--------------|-------------|
| `~mode` | `WRITE` | `WRITE` | `WRITE` records odometry; `READ` loads and executes waypoints |
| `~write_traj_file_name` | `traj.yaml` | `traj.yaml` | WRITE output file inside `config/` |
| `~read_traj_file_name` | `traj.yaml` | `traj.yaml` | READ input file inside `config/` |
| `~write_interval` | `1.0` | `5.0` | Seconds between recorded waypoints; must be greater than zero |
| `~takeoff_altitude` | `10.0` | `2.0` | Target height above the initial local z position |
| `~mission_tolerance` | `0.5` | `0.5` | 3D distance used to determine that a mission point is reached; must be greater than zero |
| `~request_interval` | `5.0` | `5.0` | Seconds between OFFBOARD/arming service attempts |
| `~init_wait_timeout` | `10.0` | `10.0` | Maximum wait for the first local-position message |

## Topics and services

| Name | Type | Direction | Purpose |
|------|------|-----------|---------|
| `/clicked_point` | `geometry_msgs/PointStamped` | Subscribe | Add an RViz target point |
| `/mavros/state` | `mavros_msgs/State` | Subscribe | Monitor FCU connection, mode, and arming state |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | Subscribe | Initialize the origin, record positions, and check target distance |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | Publish | Send local position setpoints to MAVROS |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | Service | Arm the vehicle |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | Service | Request OFFBOARD mode |

## Trajectory file format

Despite the `.yaml` extension, the trajectory currently uses a simple whitespace-separated text format:

```text
# name x y z
p0 0.000 0.000 2.000
p1 0.125 0.034 1.998
```

- Blank lines and lines beginning with `#` are ignored.
- Each data row must contain a waypoint name followed by numeric `x`, `y`, and `z` values.
- Yaw is not recorded or replayed.
- WRITE mode truncates its output file when the node starts.
- `saveTraj()` and `reverse_traj` renumber output points from `p0`.

## Safety notes

- Test READ mode and reversed trajectories in SITL before using real hardware.
- Confirm that RViz **Fixed Frame** matches the frame published by `/mavros/local_position/odom`.
- The node rejects mismatched nonempty frame IDs; it does not transform clicked points through TF.
- Starting WRITE mode truncates the selected trajectory file immediately.
- The node exits if it cannot initialize/load the trajectory or receive an initial local position.

## TODO

- [ ] READ return mode (WIP; basic waypoint playback is implemented)
- [x] Share trajectory I/O between the control and reversal nodes
- [x] Allow customizing takeoff altitude and record interval
- [x] Validate initial local position before generating setpoints
- [x] Support multiple published points in order
- [ ] Transform clicked points between different TF frames
- [ ] Merge the existing Mid360 compatibility plugin and show its point cloud in RViz
- [ ] Complete and validate READ return mode

## Tomorrow's tasks (2026-08-21)

### 1. Merge the Mid360 compatibility plugin

The Mid360 compatibility plugin has already been developed on another computer. The goal is to bring that implementation into this repository without overwriting the current trajectory and flight-control changes.

- [ ] Copy or fetch the Mid360 plugin branch/files from the other computer.
- [ ] Review its package structure, dependencies, launch files, parameters, topics, and TF frames before merging.
- [ ] Merge the plugin while preserving the current `click_to_fly`, `file_utils`, and `reverse_traj` changes.
- [ ] Add all required ROS dependencies to `package.xml` and `CMakeLists.txt`.
- [ ] Integrate the Mid360 launch configuration with the existing PX4/MAVROS and RViz launch flow.
- [ ] Confirm the LiDAR point-cloud topic, message type, timestamp, and frame ID.
- [ ] Configure RViz to display the Mid360 point cloud in the correct fixed frame.
- [ ] Build the merged workspace and resolve compiler, linker, launch, topic, and TF issues.
- [ ] Perform a short SITL/runtime smoke test and document the startup command.

### 2. Complete READ return mode

The current READ implementation loads and follows waypoint rows, but a reliable return mode needs explicit state handling and safety checks. Split the work into the following steps:

#### 2.1 Define return behavior

- [ ] Define how return mode is triggered and whether READ always means return or remains a generic playback mode.
- [ ] Decide whether the node reverses the recorded trajectory in memory or consumes a file generated by `reverse_traj`.
- [ ] Define completion behavior: hold at the home point, land, or switch to another PX4 mode.

#### 2.2 Validate trajectory and coordinate assumptions

- [ ] Reject missing, empty, malformed, or non-finite waypoint data before entering OFFBOARD mode.
- [ ] Verify that the recorded trajectory and current odometry use the same local origin and coordinate frame.
- [ ] Detect an obviously stale or shifted trajectory by comparing the current position with the expected return-path endpoint.
- [ ] Decide how much endpoint mismatch is acceptable and expose it as a parameter if needed.

#### 2.3 Prepare a safe return path

- [ ] Reverse the waypoint sequence exactly once and verify the first and final return points.
- [ ] Remove or skip duplicate neighboring points that are already within `mission_tolerance`.
- [ ] Decide whether the vehicle should first hold, climb to a safe return altitude, or move directly to the first return point.
- [ ] Preserve a safe altitude near the home point and avoid replaying noisy ground-level samples too early.

#### 2.4 Implement the return state machine

- [ ] Add explicit states such as `WAIT_FOR_ODOM`, `TAKEOFF`, `RETURNING`, `HOLDING`, `LANDING`, `COMPLETED`, and `FAILED`.
- [ ] Advance to the next waypoint only after the current waypoint is reached reliably.
- [ ] Add a per-waypoint timeout or stuck detection instead of waiting forever.
- [ ] Define behavior for OFFBOARD loss, disarming, odometry loss, and set-mode/service failures.
- [ ] Ensure the final waypoint is held instead of leaving an old or undefined setpoint active.

#### 2.5 Add parameters and launch support

- [ ] Add parameters for return altitude, endpoint-match tolerance, waypoint timeout, and final action.
- [ ] Expose WRITE/READ selection and trajectory filenames as launch arguments instead of editing the launch file manually.
- [ ] Keep parameter names consistent between code, launch files, and this README.

#### 2.6 Test and document

- [ ] Add unit tests for trajectory parsing, reversal, duplicate-point skipping, and waypoint progression.
- [ ] In SITL, record an outbound route and confirm that READ follows the reversed route back toward its start.
- [ ] Test empty files, damaged rows, origin mismatch, OFFBOARD loss, and interrupted odometry.
- [ ] Confirm that the vehicle performs the selected final action safely.
- [ ] Update the README only after the complete return flow passes the SITL test.
