# task4 - Person Following with Obstacle Avoidance (DWA)

This component implements a hierarchical controller combining proportional tracking and the Dynamic Window Approach (DWA) to track a target person while dynamically avoiding static obstacles (such as boxes, walls, and furniture).

## Architecture
- **Perception**:
  - 360° RGB Panoramic Camera for person detection via YOLO.
  - 3D LiDAR (Helios + Bpearl) with extrinsics projection and ROI point filtering.
  - Linear Kalman Filter with egomotion compensation for robust 2D position and velocity estimation of the person.
- **Controller**:
  - **Dynamic Window Approach (DWA)**: Evaluates forward circular arcs for candidate advance velocities (`0..1000` mm/s) and rotational velocities (`-1.0..1.0` rad/s).
  - **Collision Checking**: Trajectories are checked against filtered obstacle LiDAR points with robot radius and safety margins. Points belonging to the tracked person are separated and excluded from static obstacle repulsion.
  - **Cost Function**: Evaluates candidate trajectories based on:
    1. Distance to the target person.
    2. Orientation towards the person at the end of the arc.
    3. Ending the arc at nominal 1.0 m distance.
    4. Maximizing clearance to obstacles.
    5. Preferring fast forward velocity.
    6. Alignment with the baseline proportional controller.
  - **Hierarchical Safety**: Collision avoidance has strict priority over tracking. If forward movement is obstructed, the robot halts linear advance and safely pivots in-situ towards open space.

## Compilation
```bash
cmake -B build
make -C build -j8
```

## Running
```bash
bin/task4 etc/config.toml
```
