# Autonomous Aircraft Pushback & Ground Handling Simulation

A ROS 2 (Humble) simulation and visualization package for autonomous aircraft ground handling. This repository models the complex kinematics, articulation geometry, and dynamic path tracking of a **Towflexx Clamping Tug** manipulating an **ATR-42 Regional Aircraft** inside an airport hangar environment.

---

## Overview & Capabilities

This package translates raw trajectory planning data (`trajectories.csv`) into a high-fidelity 3D visualization using `RViz2`, `robot_state_publisher`, and custom ROS 2 marker arrays. It allows researchers and engineers to visually validate trajectory planners and tracking controllers by projecting predicted dynamic paths against ideal kinematic reference baselines.

### Key Features
* **Dual-Mode Simulation Architecture:**
  * **Combined Articulated System:** Simulates the Towflexx tug clamping the ATR-42 nose wheel, tracking both active dynamic steering and global reference corridors across all landing gears.
  * **Standalone Tug Mode:** Decouples the Towflexx tug for independent controller validation, steering limit verification, and chassis kinematics without aircraft visual clutter.
* **Unified Control Point Kinematics:** Simplifies vehicle tracking by collapsing the tug rear axle center, clamping cradle rotation center, and base link origin into a single unified coordinate $(0,0,0)$.
* **Table-Compliant High-Contrast Styling:** Implements a strict, multi-layer visual hierarchy in RViz to eliminate Z-fighting against light-colored hangar occupancy grids.
* **Predictive Dotted Lookahead:** Dynamically projects the vehicle's planned trajectory exactly **30 time steps (~1.5 seconds at 20Hz)** ahead of the rolling tires.


## Repository Structure

```text
aircraft_design/
├── aircraft_design/
│   ├── __init__.py
│   ├── csv_trajectory_player.py          # Node: Combined system trajectory & marker broadcaster
│   ├── tug_trajectory_player.py          # Node: Standalone tug trajectory & marker broadcaster
│   ├── hangar_map_publisher.py           # Node: OccupancyGrid hangar floor map publisher
│   ├── hangar_dynamic_planner.py         # Node: Dynamic path planning utility
│   └── physics_engine/                   # Submodule: Dynamic simulation & kinematics utilities
├── data/
│   ├── trajectories.csv                  # Active master dataset (kin_ and dyn_ columns)
│   ├── towing_path_halle_straight_expand_new.csv # Straight corridor trajectory logs
│   ├── towing_path_halle_turn_expand_new.csv     # Turning corridor trajectory logs
│   └── dynamic_pushback_log.csv          # Recorded pushback telemetry
├── launch/
│   ├── hangar_simulation.launch.py       # Launch: Combined pushback (Tug + ATR-42 + Map)
│   ├── hangar_tug_simulation.launch.py   # Launch: Standalone Tug + Map tracking
│   ├── display_tug.launch.py             # Launch: Standalone Tug CAD & joint GUI inspector
│   └── system.launch.py                  # Launch: Raw state publisher (Combined System)
├── maps/
│   ├── halle_straight_map.png / .yaml    # Hangar straight hallway occupancy grid maps
│   ├── test_turn_map.png / .yaml         # Hangar turning corridor occupancy grid maps
│   └── airport_map.png / .yaml           # Full airport tarmac layout
├── meshes/
│   ├── tractor_base.stl                  # Towflexx chassis & drive assembly
│   ├── tractor_hitch.stl                 # Rotating clamping cradle
│   ├── rear_wheel_group.stl              # Drive wheel geometry
│   ├── aircraft_base.stl                 # ATR-42 fuselage & wings
│   ├── aircraft_nose_wheel.stl           # Nose landing gear assembly
│   ├── aircraft_left_mlg.stl             # Left main landing gear
│   └── aircraft_right_mlg.stl            # Right main landing gear
├── rviz/
│   ├── pushback.rviz                     # RViz config: Combined aircraft pushback
│   ├── tug_simulation.rviz               # RViz config: Standalone tug tracking
│   └── display_tug.rviz                  # RViz config: Standalone tug CAD inspection
├── urdf/
│   ├── system.urdf.xacro                 # Combined Articulated Vehicle URDF
│   ├── aircraft.urdf.xacro               # ATR-42 geometry & link definitions
│   ├── tractor.urdf.xacro                # Legacy Towflexx Tug URDF definitions
│   └── tug.urdf.xacro                    # Standalone Towflexx Tug URDF
├── package.xml
└── setup.py

```

## Trajectory & Marker Hierarchy

To maximize visual separation and prevent line blending, all paths are separated by Z-elevation:

•⁠  ⁠Reference corridors: ⁠ Z = 0.20 m ⁠
•⁠  ⁠Dynamic lookahead blocks: ⁠ Z = 0.30 m ⁠

The visualization follows a bold, high-contrast color palette for better readability in RViz.

---

## 1. Combined Articulated System

Node used: ⁠ csv_trajectory_player ⁠

| Marker Content | Marker ID | Line Style | Line Width | Assigned Color | RGB Values ⁠ (r, g, b) ⁠ |
| :--- | :--- | :--- | :--- | :--- | :--- |
| *Planned Center Axle: 30-Step* | ⁠ 0 ⁠ | *Dotted* | ⁠ 0.50 m ⁠ | *Deep Blue* | ⁠ (0.0, 0.10, 0.95) ⁠ |
| *Planned Left MLG: 30-Step* | ⁠ 1 ⁠ | *Dotted* | ⁠ 0.50 m ⁠ | *Electric Sky Blue* | ⁠ (0.0, 0.75, 1.00) ⁠ |
| *Planned Right MLG: 30-Step* | ⁠ 2 ⁠ | *Dotted* | ⁠ 0.50 m ⁠ | *Electric Sky Blue* | ⁠ (0.0, 0.75, 1.00) ⁠ |
| *Reference Center Axle* | ⁠ 10 ⁠ | *Solid* | ⁠ 0.25 m ⁠ | *Emerald Green* | ⁠ (0.0, 0.55, 0.15) ⁠ |
| *Reference Left MLG* | ⁠ 11 ⁠ | *Solid* | ⁠ 0.25 m ⁠ | *Crisp Mint Green* | ⁠ (0.25, 0.95, 0.25) ⁠ |
| *Reference Right MLG* | ⁠ 12 ⁠ | *Solid* | ⁠ 0.25 m ⁠ | *Crisp Mint Green* | ⁠ (0.25, 0.95, 0.25) ⁠ |
| *Start Pose: Nose Hitch* | ⁠ 20 ⁠ | *Arrow* | ⁠ 3.00 m ⁠ | *Bold Yellow* | ⁠ (1.0, 0.90, 0.0) ⁠ |
| *End Pose: Nose Hitch* | ⁠ 21 ⁠ | *Arrow* | ⁠ 3.00 m ⁠ | *Bold Red* | ⁠ (0.95, 0.10, 0.10) ⁠ |

---

## 2. Standalone Tug Simulation

Node used: ⁠ tug_trajectory_player ⁠

| Marker Content | Marker ID | Line Style | Line Width | Assigned Color | RGB Values ⁠ (r, g, b) ⁠ |
| :--- | :--- | :--- | :--- | :--- | :--- |
| *Planned Tug Center: 30-Step* | ⁠ 0 ⁠ | *Dotted* | ⁠ 0.50 m ⁠ | *Dark Blue* | ⁠ (0.0, 0.15, 0.85) ⁠ |
| *Global Reference Tug Center* | ⁠ 10 ⁠ | *Solid* | ⁠ 0.25 m ⁠ | *Solid Green* | ⁠ (0.0, 0.70, 0.15) ⁠ |
| *Start / End Poses* | ⁠ 20 ⁠ / ⁠ 21 ⁠ | *Arrow* | ⁠ 2.50 m ⁠ | *Yellow / Red* | ⁠ (1.0, 0.9, 0.0) ⁠ / ⁠ (0.95, 0.1, 0.1) ⁠ |

---

## Dotted Lookahead Logic

Dotted trajectories use ⁠ LINE_LIST ⁠ markers and are generated by slicing the future CSV horizon from:

⁠ text
idx to idx + 30
 ⁠

The marker generation steps through the array in increments of ⁠ 3 ⁠. This means the visualization draws one row and skips two rows, resulting in *10 bold half-meter-wide tarmac blocks* floating above the green reference baseline.

---
## Build & Installation

### 1. Source the ROS 2 Environment

⁠ bash
source /opt/ros/humble/setup.bash
 ⁠

### 2. Navigate to the Workspace

⁠ bash
cd ~/flight_ws
 ⁠

### 3. Build the Package

⁠ bash
colcon build --symlink-install
 ⁠

### 4. Source the Workspace

⁠ bash
source install/setup.bash
 ⁠

---

## Launch Commands

### 1. Full Hangar Pushback Simulation

This launch file spawns the ATR-42 clamped into the Towflexx tug, broadcasts the hangar floor map, and animates the vehicle along the 30-step predictive corridor.

⁠```bash
ros2 launch aircraft_design hangar_simulation.launch.py
 ⁠

---

### 2. Standalone Tug Simulation

This launch file spawns only the Towflexx tug on the hangar map. It broadcasts the unified center axle trajectory without aircraft mesh interference.

⁠```bash
ros2 launch aircraft_design hangar_tug_simulation.launch.py
 ⁠

---

### 3. Standalone Tug CAD & Joint Inspector

This launch file opens an isolated inspection setup with a GUI slider using ⁠ joint_state_publisher_gui ⁠. It is used to manually test the clamping cradle rotation limits of approximately ⁠ ±90° ⁠.

⁠```bash
ros2 launch aircraft_design display_tug.launch.py
 ⁠

---

### 4. Combined System CAD Inspector

This launch file opens the combined articulated aircraft and tug assembly in an isolated RViz view.

⁠```bash
ros2 launch aircraft_design system.launch.py
 ⁠

---

## Notes

•⁠  ⁠Make sure the package is built successfully before launching any simulation.
•⁠  ⁠Always source both ROS 2 Humble and the workspace before running launch commands.
•⁠  ⁠The main trajectory input file is located at:

⁠```text
data/trajectories.csv
 ⁠

•⁠  ⁠The RViz configuration files are located in:

⁠```text
rviz/
