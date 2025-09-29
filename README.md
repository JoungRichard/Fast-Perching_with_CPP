# Trajectory Optimizer

A ROS-independent trajectory optimization library for UAV perching maneuvers.

## Overview

This project provides trajectory optimization capabilities for UAV perching, extracted from the original ROS-based Fast-Perching implementation. The library provides:

- **TrajOpt**: The original class interface, cleaned of ROS dependencies for standalone use

## Features

- **ROS-Independent**: Complete removal of ROS dependencies for use in any C++ project
- **MINCO Trajectory Generation**: Minimum control effort trajectory optimization
- **L-BFGS Optimization**: Efficient gradient-based optimization
- **Collision Avoidance**: Built-in collision detection and avoidance constraints
- **Dynamic Constraints**: Velocity, acceleration, thrust, and angular velocity limits
- **Perching Optimization**: Specialized for UAV perching maneuvers

## Dependencies

- **Eigen3**: Linear algebra library
- **C++14**: Minimum C++ standard required

## Building
Windows:
```bash
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make -j8
```

## Usage

### Basic Example

```cpp
#include "traj_opt.h"

// Create optimizer
traj_opt::TrajOpt optimizer;

// Configure parameters
optimizer.setDynamicLimits(10.0, 10.0, 20.0, 2.0, 3.0, 2.0);
optimizer.setRobotParameters(1.0, 0.3, 0.1, 0.5);
optimizer.setOptimizationWeights(1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);

// Set initial state
Eigen::MatrixXd initial_state(3, 4);
initial_state.col(0) = Eigen::Vector3d(0.0, 0.0, 2.0);  // position
initial_state.col(1) = Eigen::Vector3d(5.0, 0.0, 0.0);  // velocity

// Set target
Eigen::Vector3d target_position(10.0, 0.0, 1.0);
Eigen::Vector3d target_velocity(2.0, 0.0, 0.0);
Eigen::Quaterniond landing_quaternion(1.0, 0.0, 0.0, 0.0);

// Generate trajectory
Trajectory result_trajectory;
bool success = optimizer.generate_traj(
    initial_state, target_position, target_velocity, 
    landing_quaternion, 10, result_trajectory
);
```

### Configuration Methods

#### Dynamic Limits
```cpp
optimizer.setDynamicLimits(
    max_velocity,        // m/s
    max_acceleration,    // m/s²
    max_thrust,         // N
    min_thrust,         // N
    max_angular_velocity,    // rad/s
    max_yaw_angular_velocity // rad/s
);
```

#### Robot Parameters
```cpp
optimizer.setRobotParameters(
    velocity_plus,   // Landing velocity offset
    robot_length,    // Robot length (m)
    robot_radius,    // Robot radius (m)
    platform_radius  // Landing platform radius (m)
);
```

#### Optimization Weights
```cpp
optimizer.setOptimizationWeights(
    time_weight,              // Time penalty weight
    velocity_tail_weight,     // Terminal velocity weight
    position_weight,          // Position constraint weight
    velocity_weight,          // Velocity constraint weight
    acceleration_weight,      // Acceleration constraint weight
    thrust_weight,           // Thrust constraint weight
    angular_velocity_weight, // Angular velocity constraint weight
    perching_collision_weight // Collision avoidance weight
);
```

## Code Structure

```
├── include/
│   ├── traj_opt.h              # Main trajectory optimizer interface
│   ├── minco.hpp               # MINCO trajectory generation
│   ├── lbfgs_raw.hpp          # L-BFGS optimization
│   ├── poly_traj_utils.hpp    # Polynomial trajectory utilities
│   └── root_finder.hpp        # Root finding utilities
├── src/
│   └── traj_opt_perching.cc   # Implementation
├── examples/
│   ├── basic_example.cpp      # Basic usage example
│   └── precision_test.cpp     # Precision testing
├── scripts/
│   ├── visualize_trajectories.py  # Trajectory visualization
│   └── trajectory_summary.py      # Trajectory analysis
└── CMakeLists.txt             # Build configuration
```

## Key Features

- **ROS-Independent**: Complete elimination of `ros::NodeHandle` and ROS-specific code
- **Clean Interface**: Simplified API with clear setter methods
- **Visualization Tools**: Python scripts for trajectory analysis and visualization
- **Example Code**: Working examples demonstrating usage
- **MINCO Optimization**: Minimum control effort trajectory generation

## Original Attribution

This code is derived from the Fast-Perching project:
- **Repository**: [ZJU-FAST-Lab/Fast-Perching](https://github.com/ZJU-FAST-Lab/Fast-Perching)
- **Original Authors**: ZJU FAST Lab

## License

Please refer to the original Fast-Perching repository for licensing information.