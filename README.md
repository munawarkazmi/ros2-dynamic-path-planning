# ROS 2 Dynamic Path Planning  
**A* vs D* Lite Global Planners for Nav2 – Real-Time Replanning in Dynamic Environments**

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C++-20-blue)](https://en.cppreference.com/w/cpp/20)

**D* Lite is 22.4% faster on average** than A* in changing environments  
→ proven with **200 fully reproducible trials** (seed = 42)

Zero hacks · Pure Nav2 plugins · One-command benchmark · Live moving obstacles

> D* Lite smoothly adapts when people walk into the path.  
> A* throws away everything and replans from scratch.

## Results – 200 Trials (seed 42)

```text
A* average time      : 3.298 s
D* Lite average time : 2.560 s
→ D* Lite is 22.37% faster
D* Lite won 197 / 200 trials
Full data → reports/results/travel_time_comparison.csv
Environment
50 m × 40 m indoor office · 0.05 m resolution · 8 moving people
Indoor Office Map → maps/indoor_grid.png
Quick Start (ROS 2 Humble / Iron / Jazzy)
Bashgit clone https://github.com/munawarkazmi/ros2-dynamic-path-planning.git
cd ros2_ws/src && ln -s ../ros2-dynamic-path-planning . && cd ../..
colcon build --packages-select ros2_dynamic_path_planning
source install/setup.bash

# 1. Live demo with D* Lite (RViz auto-opens)
ros2 launch ros2_dynamic_path_planning dstar_lite_launch.py rviz:=true

# 2. Live demo with classic A*
ros2 launch ros2_dynamic_path_planning astar_launch.py rviz:=true

# 3. Reproduce the full 200-trial benchmark (~2 min)
ros2 launch ros2_dynamic_path_planning benchmark_launch.py trials:=200 seed:=42
Features

Modern C++20 implementation
Real Nav2 global planner plugins (pluginlib)
Dynamic obstacles via MarkerArray
Single-parameter planner switching
100% reproducible benchmark node with CSV output
Production-grade launch files & configs
Pixel-perfect Gazebo world with 8 walking actors (in worlds/)

Project Structure
textros2_dynamic_path_planning/
├── src/          → A*, D* Lite, wrapper, benchmark node
├── include/      → Headers
├── config/       → nav2_params.yaml, costmap_params.yaml
├── launch/       → *_launch.py files
├── maps/         → indoor_grid.png + .yaml (50×40 m)
├── rviz/         → nav2_config.rviz (perfect view)
├── worlds/       → dynamic_indoor.world (8 moving people)
└── reports/results/ → 200-trial proof (CSV)
Why This Matters
In real warehouses, hospitals, and homes the map changes constantly.
Traditional A* replans from scratch → high CPU spikes, jerky motion.
D* Lite reuses previous search results → smooth, fast, low latency.
This repository proves it — with code you can run today.
License
MIT © 2025 Munawar Kazmi

Star if this helped you ship faster, smoother robots
