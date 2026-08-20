# click_to_fly

Click-to-fly control for a PX4 drone (SITL) via MAVROS: click a point in RViz and the drone flies there at a fixed altitude in OFFBOARD mode.

## Features

- Take off and fly to a clicked point (`/clicked_point`) at **2 m** altitude
- Automatic OFFBOARD mode + arming via MAVROS services
- Two modes:
  - **WRITE** (default): records the flown trajectory (x, y, z, yaw) to `config/traj.yaml` every second (file is truncated on start)
  - **READ**: loads a recorded trajectory from `config/traj.yaml` and logs the waypoints (playback WIP)
- Launch file starts the node together with RViz (auto-loads a preconfigured `.rviz` with drone odometry, clicked-point marker, TF and Grid) and rqt_console

## Repository structure

```
click_to_fly/
├── src/click_to_fly.cpp      # main node
├── launch/click_to_fly.launch # node + rviz + rqt_console
├── config/
│   ├── traj.yaml             # recorded / loaded trajectory
│   └── click_to_fly.rviz     # RViz config (auto-loaded)
├── CMakeLists.txt
└── package.xml
```

## Test Environment

| Component | Version |
|-----------|---------|
| ROS | Melodic / Noetic (ROS 1) |
| PX4 | 1.x SITL (jMAVSim) |
| MAVROS | `mavros_posix_sitl.launch` |

## Prerequisites

- Ubuntu 18.04/20.04 with ROS Melodic/Noetic installed
- PX4 SITL environment (`px4`, `jMAVSim`, `mavros`)
- RViz, `rqt_console`
- `catkin_make`

## Compile

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Run

### Terminal 1 - PX4 SITL

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch px4 mavros_posix_sitl.launch
```

### Terminal 2 - click_to_fly

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch click_to_fly click_to_fly.launch            # default: WRITE mode
roslaunch click_to_fly click_to_fly.launch _mode:=WRITE _traj_file_name:=traj.yaml
roslaunch click_to_fly click_to_fly.launch _mode:=READ _traj_file_name:=traj.yaml
```

(`_param:=value` sets the node's private parameters.)

## Usage

1. Start Terminal 1, wait for SITL, then start the package in Terminal 2.
2. Wait until the log shows `FCU connected` and the vehicle is armed in OFFBOARD mode.
3. In RViz (already configured by the launch file), use the **Publish Point** tool to click a location on the ground. The drone takes off and flies to the clicked location.
4. In WRITE mode the flown trajectory is saved to `config/traj.yaml`; load it later with READ mode.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `~mode` | `WRITE` | `WRITE` records trajectory, `READ` loads it |
| `~traj_file_name` | `traj.yaml` | Trajectory file in `config/` |
| `~request_interval` | `5.0` | Interval (s) between OFFBOARD/arm service attempts |
| `~init_wait_timeout` | `10.0` | Max time (s) to wait for initial local position |

## Topics / Services

| Type | Name | Direction |
|------|------|-----------|
| `/clicked_point` | `geometry_msgs/PointStamped` | Subscribed - clicked target in RViz |
| `/mavros/state` | `mavros_msgs/State` | Subscribed - FCU state |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | Subscribed - local position |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | Published - OFFBOARD setpoint (FRAME_LOCAL_NED) |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | Service - arming |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | Service - set OFFBOARD mode |

## Trajectory file format

```
# name x y z yaw
p0 0.000 0.000 2.000 0.000
p1 0.125 0.034 1.998 0.012
...
```

Lines starting with `#` and empty lines are ignored.

## TODO

- [ ] Playback of recorded waypoints in READ mode
- [x] Allow customizing takeoff altitude
- [ ] Allow customizing takeoff spawn point
- [x] Move configurable values (e.g. altitude, record frequency) into the launch file
- [ ] Improve positioning precision
- [x] Provide a default `.rviz` config that adds topics automatically
- [x] Drop yaw from the recorded trajectory
- [ ] Add Mid360 lidar and show point cloud in RViz for manual obstacle avoidance
- [x] Support multiple published points
- [ ] Support multiple published points, arrive in order using a B-spline path