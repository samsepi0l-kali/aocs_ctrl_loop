# Spacecraft Attitude Control Simulation

This project simulates the attitude control of a rigid-body spacecraft using a PID controller. The spacecraft's orientation is modeled using a small-angle approximation of the attitude error vector, and stabilized using control torques computed by the PID controller.

## File

- `acl.cpp`: Contains the full implementation of the simulation.

## Features

- 3D vector and matrix operations
- Rigid-body attitude dynamics with a diagonal inertia tensor
- PID controller with integral windup protection
- Simulation loop with disturbance injection
- Console output of attitude error norm and angular velocity

## Simulation Description

- The spacecraft begins at rest.
- A disturbance is applied at step 10.
- The PID controller reacts to bring the system back to the target orientation.
- Outputs the attitude error norm and angular velocity at each step.

Example output:
```
Step 10 | Error: 0.173205 | w: (0.01, 0.01, 0.02)
Step 11 | Error: 0.150102 | w: (0.019, 0.019, 0.038)
...
```

## Build and Run

Compile and run using:

```bash
g++ -std=c++11 acl.cpp -o acl
./acl
```
