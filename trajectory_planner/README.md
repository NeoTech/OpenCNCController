# Trajectory Planner Library

A high-performance trajectory planning library for CNC motion control.

## Features

- **Look-ahead Planning**: Configurable depth (default 32 segments)
- **Velocity Profiles**: Trapezoidal and S-curve (jerk-limited)
- **Corner Blending**: Junction deviation-based velocity limiting
- **Arc Interpolation**: Automatic linearization with tolerance control
- **Thread-Safe**: Lock-free ring buffer for real-time operation
- **Multi-Axis**: Support for up to 9 axes (X, Y, Z, A, B, C, U, V, W)

## Architecture

```
                    ┌─────────────────┐
    G-code ──────►  │  Motion Queue   │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │ Forward Pass    │  Calculate max entry velocities
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │ Backward Pass   │  Apply deceleration limits
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │ Profile Gen     │  Create velocity profiles
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
    Position ◄────  │ Interpolation   │  Real-time position output
                    └─────────────────┘
```

## Usage

```cpp
#include <traj_planner.h>

using namespace traj_planner;

// Create planner with configuration
TrajectoryPlanner planner;
PlannerConfig config;

// Set axis constraints
config.axes[0].max_velocity = 5000;      // mm/min
config.axes[0].max_acceleration = 500;    // mm/s²
config.axes[1] = config.axes[0];
config.axes[2].max_velocity = 2000;       // Z is slower

config.junction_deviation = 0.05;         // mm
config.lookahead_depth = 32;

planner.setConfig(config);

// Add moves
Position target1, target2;
target1[0] = 100.0;  // X = 100
target1[1] = 50.0;   // Y = 50

planner.addLinearMove(target1, 3000);  // Feed 3000 mm/min
planner.addLinearMove(target2, 3000);

// Plan the trajectory
planner.plan();

// Start execution
planner.start();

// Control loop (call at fixed rate, e.g., 1kHz)
while (planner.isRunning()) {
    MotionState state = planner.update(0.001);  // 1ms timestep
    
    // state.position contains interpolated position
    // state.velocity contains current velocity
    
    // Output to step generators...
}
```

## Velocity Profile Types

### Trapezoidal (3-phase)
```
  v ▲
    │     ┌──────────┐
    │    /            \
    │   /              \
    │  /                \
    └──────────────────────► t
       accel  cruise  decel
```

### S-Curve (7-phase, jerk-limited)
```
  v ▲
    │       ╭──────╮
    │      ╱        ╲
    │     ╱          ╲
    │    ╱            ╲
    └──────────────────────► t
      Smoother acceleration
```

## Junction Velocity

The planner uses junction deviation to calculate safe corner velocities:

```
    Segment 1 ───────►
                      ╲
                       ╲  Junction
                        ╲
                         ▼ Segment 2
```

Junction velocity is limited based on:
- Angle between segments
- Maximum centripetal acceleration
- Configured junction deviation tolerance

## Thread Safety

The motion queue uses lock-free atomic operations for the real-time path:
- `update()` can be called from a high-priority thread
- Adding moves is done from a lower-priority thread
- Planning is done when queue is modified

## Performance Considerations

- Pre-allocate queue capacity to avoid runtime allocations
- Use `update()` with fixed timestep for deterministic behavior
- Larger lookahead depth = smoother motion but more planning time
- S-curve profiles are computationally heavier than trapezoidal
