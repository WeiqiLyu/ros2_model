# Autonomous Aircraft Pushback & Ground Handling Simulation

A ROS 2 Humble simulation and visualization package for autonomous aircraft ground handling.

This repository models the kinematics, articulation geometry, and dynamic path tracking of a **Towflexx Clamping Tug** manipulating an **ATR-42 Regional Aircraft** inside an airport hangar environment.

---

## Overview

This package converts trajectory planning data from `trajectories.csv` into a 3D visualization using:

* `RViz2`
* `robot_state_publisher`
* Custom ROS 2 marker arrays
* URDF/Xacro vehicle models
* Hangar occupancy grid maps

The simulation helps visualize and validate aircraft pushback trajectories, tug motion, steering behavior, and reference path tracking.

---

## Key Features

* **Combined Aircraft + Tug Simulation**
  * Simulates the Towflexx tug clamping the ATR-42 nose wheel.
  * Tracks dynamic and reference trajectories for aircraft landing gear positions.

* **Standalone Tug Simulation**
  * Simulates only the Towflexx tug.
  * Useful for validating tug controller behavior without aircraft mesh interference.

* **Predictive Lookahead Path**
  * Shows the planned trajectory **30 time steps ahead**.
  * At 20 Hz, this corresponds to approximately **1.5 seconds** of future motion.

* **RViz Marker Visualization**
  * Displays planned paths, reference paths, start pose, and end pose.
  * Uses high-contrast colors for better visibility on hangar maps.

---

## Repository Structure

```text
aircraft_design/
├── aircraft_design/
│   ├── __init__.py
│   ├── csv_trajectory_player.py          # Node: Combined system trajectory and marker broadcaster
│   ├── tug_trajectory_player.py          # Node: Standalone tug trajectory and marker broadcaster
│   ├── hangar_map_publisher.py           # Node: OccupancyGrid hangar floor map publisher
│   ├── hangar_dynamic_planner.py         # Node: Dynamic path planning utility
│   └── physics_engine/                   # Submodule: Dynamic simulation and kinematics utilities
├── data/
│   ├── trajectories.csv                  # Active master dataset with kin_ and dyn_ columns
│   ├── towing_path_halle_straight_expand_new.csv
│   ├── towing_path_halle_turn_expand_new.csv
│   └── dynamic_pushback_log.csv
├── launch/
│   ├── hangar_simulation.launch.py       # Launch: Combined pushback, tug, ATR-42, and map
│   ├── hangar_tug_simulation.launch.py   # Launch: Standalone tug and map tracking
│   ├── display_tug.launch.py             # Launch: Standalone tug CAD and joint GUI inspector
│   └── system.launch.py                  # Launch: Raw state publisher for combined system
├── maps/
│   ├── halle_straight_map.png
│   ├── halle_straight_map.yaml
│   ├── test_turn_map.png
│   ├── test_turn_map.yaml
│   ├── airport_map.png
│   └── airport_map.yaml
├── meshes/
│   ├── tractor_base.stl
│   ├── tractor_hitch.stl
│   ├── rear_wheel_group.stl
│   ├── aircraft_base.stl
│   ├── aircraft_nose_wheel.stl
│   ├── aircraft_left_mlg.stl
│   └── aircraft_right_mlg.stl
├── rviz/
│   ├── pushback.rviz
│   ├── tug_simulation.rviz
│   └── display_tug.rviz
├── urdf/
│   ├── system.urdf.xacro                 # Combined articulated vehicle URDF
│   ├── aircraft.urdf.xacro               # ATR-42 geometry and link definitions
│   ├── tractor.urdf.xacro                # Legacy Towflexx tug URDF definitions
│   └── tug.urdf.xacro                    # Standalone Towflexx tug URDF
├── package.xml
└── setup.py
```

---

## Main ROS 2 Nodes

### `csv_trajectory_player.py`

Used for the **combined aircraft and tug simulation**.

It publishes:
* Aircraft and tug motion
* Dynamic planned trajectory
* Reference trajectory
* Start and end pose markers
* Landing gear path markers

### `tug_trajectory_player.py`

Used for the **standalone tug simulation**.

It publishes:
* Tug motion
* Tug center trajectory
* Reference tug path
* Start and end pose markers

### `hangar_map_publisher.py`

Publishes the hangar map as an occupancy grid for RViz visualization.

### `hangar_dynamic_planner.py`

Contains dynamic path planning utilities for pushback trajectory generation.

---

## Trajectory & Marker Hierarchy

To improve visibility in RViz, paths are separated by height:

* **Reference paths:** `Z = 0.20 m`
* **Dynamic lookahead paths:** `Z = 0.30 m`

This prevents visual overlap and line blending with the hangar floor map.

---

## Combined Aircraft + Tug Markers

**Node used:** `csv_trajectory_player.py`

| Marker Content | Marker ID | Line Style | Line Width | Color |
| :--- | :--- | :--- | :--- | :--- |
| Planned Center Axle | `0` | Dotted | `0.50 m` | Deep Blue |
| Planned Left Main Landing Gear | `1` | Dotted | `0.50 m` | Electric Sky Blue |
| Planned Right Main Landing Gear | `2` | Dotted | `0.50 m` | Electric Sky Blue |
| Reference Center Axle | `10` | Solid | `0.25 m` | Emerald Green |
| Reference Left Main Landing Gear | `11` | Solid | `0.25 m` | Mint Green |
| Reference Right Main Landing Gear | `12` | Solid | `0.25 m` | Mint Green |
| Start Pose | `20` | Arrow | `3.00 m` | Yellow |
| End Pose | `21` | Arrow | `3.00 m` | Red |

---

## Standalone Tug Markers

**Node used:** `tug_trajectory_player.py`

| Marker Content | Marker ID | Line Style | Line Width | Color |
| :--- | :--- | :--- | :--- | :--- |
| Planned Tug Center | `0` | Dotted | `0.50 m` | Dark Blue |
| Reference Tug Center | `10` | Solid | `0.25 m` | Solid Green |
| Start Pose | `20` | Arrow | `2.50 m` | Yellow |
| End Pose | `21` | Arrow | `2.50 m` | Red |

---

## Dotted Lookahead Logic

The dotted trajectory uses future CSV points from the current index: `idx` to `idx + 30`.

The marker publisher skips every few rows to create a dotted effect. This creates approximately **10 visible blocks** representing the future path of the vehicle.

---

## Build & Installation

### 1. Source the ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

---

### 2. Navigate to the Workspace

```bash
cd ~/flight_ws
```

---

### 3. Build the Package

```bash
colcon build --symlink-install
```

---

### 4. Source the Workspace

```bash
source install/setup.bash
```

---

## Launch Commands

### 1. Full Hangar Pushback Simulation

This launch file spawns the ATR-42 clamped into the Towflexx tug, broadcasts the hangar floor map, and animates the vehicle along the 30-step predictive corridor.

```bash
ros2 launch aircraft_design hangar_simulation.launch.py
```

---

### 2. Standalone Tug Simulation

This launch file spawns only the Towflexx tug on the hangar map. It broadcasts the unified center axle trajectory without aircraft mesh interference.

```bash
ros2 launch aircraft_design hangar_tug_simulation.launch.py
```

---

### 3. Standalone Tug CAD & Joint Inspector

This launch file opens an isolated inspection setup with a GUI slider using `joint_state_publisher_gui`. It is used to manually test the clamping cradle rotation limits of approximately ±90°.

```bash
ros2 launch aircraft_design display_tug.launch.py
```

---

### 4. Combined System CAD Inspector

This launch file opens the combined articulated aircraft and tug assembly in an isolated RViz view.

```bash
ros2 launch aircraft_design system.launch.py
```

---

## Notes

* Make sure the package is built successfully before launching any simulation.
* Always source both ROS 2 Humble and the workspace before running launch commands.
* The main trajectory input file is located at:

```text
data/trajectories.csv
```

* The RViz configuration files are located in:

```text
rviz/
```

```