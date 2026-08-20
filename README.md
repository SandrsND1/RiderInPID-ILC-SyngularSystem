# RiderInPID-ILC Syngular System

Unity-based vehicle simulation implementing PID control and Iterative Learning Control (ILC) for autonomous track following.

The project combines a kinematic bicycle vehicle model, PID feedback control, Cross-Track Error (CTE) correction, and iterative learning between laps.

## Features

- PID steering controller
- Iterative Learning Control (ILC)
- Kinematic bicycle vehicle model
- Cross-Track Error (CTE) calculation
- Lookahead-based path following
- Steering rate limiting
- Integral anti-windup
- Filtered derivative term
- Multi-lap simulation
- Lap time measurement
- Runtime controller debugging
- Simulation data recording

## Architecture

```text
TrackPath
    │
    ▼
TrackFollower
    │
    ├── Cross-Track Error
    ├── Lookahead Point
    └── Target Heading
            │
            ▼
      SingularPID_ILC
            │
            ├── P Term
            ├── I Term
            ├── D Term
            └── ILC Correction
                    │
                    ▼
          SingularVehicleModel
                    │
                    ▼
              Vehicle State
                    │
                    └──────► TrackFollower
```
PID Controller

The controller calculates the steering command using proportional, integral, and derivative terms.

u_PID = Kp * e + Ki * ∫e dt + Kd * de/dt

Where:

e — heading error
Kp — proportional gain
Ki — integral gain
Kd — derivative gain
Proportional Term
P = Kp * headingError

Default:

Kp = 3.5
Integral Term

The integral component reduces persistent tracking error.

An anti-windup mechanism limits the accumulated integral error.

Ki = 0.08
maxIntegral = 0.4
Derivative Term

The derivative component helps reduce oscillations.

The derivative signal is filtered before being applied.

Kd = 0.9
derivativeFilterK = 0.7
Iterative Learning Control

ILC improves the steering command using information collected from previous laps.

During the first lap, the controller records the Cross-Track Error.

On subsequent laps, the stored error is used to update the steering correction:

ILC[k] = ILC[k] - learningRate * previousCTE[k]

The correction is limited to prevent unstable steering commands.

Default parameters:

learningRate = 0.02
maxILC = 0.16 rad

The ILC learning process starts after the first completed lap.

Vehicle Model

The vehicle uses a simplified kinematic bicycle model.

State Variables
x     - X position
y     - Y position
theta - vehicle heading
delta - steering angle
v     - vehicle velocity
Vehicle Dynamics
dx     = v * cos(theta)
dy     = v * sin(theta)
dtheta = (v / L) * tan(delta)

Default wheelbase:

L = 2.5 m

The vehicle model also:

limits steering angle
limits velocity
stabilizes steering dynamics
integrates vehicle position
normalizes heading to [-π, π]
Track Following

TrackFollower calculates the vehicle's position relative to the track.

It provides:

nearest track point
lookahead point
Cross-Track Error
target heading
Cross-Track Error

The vehicle position is projected onto the nearest track segment.

The distance from the vehicle to the projected point represents the tracking error.

The sign of the error is determined using the track normal.

CTE > 0 → vehicle is on one side of the track
CTE < 0 → vehicle is on the opposite side
Lookahead

The controller uses a point ahead of the vehicle to determine the desired direction of travel.

Default:

lookaheadDistance = 6 m

Additional correction is applied when the vehicle moves significantly away from the track.

Steering Constraints

The steering angle is limited:

maxSteering = 0.75 rad

The rate of steering change is also limited:

steeringRate = 6 rad/s

This prevents instantaneous steering changes and produces smoother vehicle behavior.

Race System

RaceManager controls the multi-lap simulation.

Default configuration:

totalLaps = 5

The system records:

lap time
current lap
best lap
average lap time

After the final lap, the vehicle is stopped and race statistics are printed to the Unity console.

Data Recording

DataRecorder collects simulation data for analysis.

Available controller data includes:

vehicle position
vehicle velocity
steering angle
PID P term
PID I term
PID D term
integral error
heading error
Cross-Track Error
ILC correction
ILC index
nearest track index

The recorded data can be used to evaluate controller performance and compare tracking behavior across laps.

Project Structure
```text
Assets/
├── Action Map/
│   ├── CarControlActions.cs
│   └── CarControlActions.inputactions
│
├── Scenes/
│   └── SampleScene.unity
│
├── Scripts/
│   ├── Control/
│   │   ├── SingularPID_ILC.cs
│   │   └── SingularVehicleModel.cs
│   │
│   ├── Simulation/
│   │   ├── DataRecorder.cs
│   │   └── RaceManager.cs
│   │
│   ├── Track/
│   │   ├── TrackFollower.cs
│   │   ├── TrackPath.cs
│   │   └── TrackSetup.cs
│   │
│   ├── Visualization/
│   │   ├── CarVisual.cs
│   │   └── TrackRenderer.cs
│   │
│   ├── CameraFollow.cs
│   └── VehicleModelHolder.cs
│
├── Material/
├── Settings/
└── ...
```
Main Components
SingularPID_ILC

Main control system responsible for:

PID control
ILC learning
steering control
speed control
controller state management
lap tracking
SingularVehicleModel

Mathematical vehicle model responsible for:

vehicle state
vehicle kinematics
steering dynamics
position integration
TrackFollower

Track tracking subsystem responsible for:

nearest point detection
CTE calculation
lookahead calculation
target heading calculation
TrackPath

Stores the track geometry and provides:

nearest point search
track direction
track visualization
RaceManager

Controls:
multi-lap simulation
lap timing
race completion
race statistics

DataRecorder
Collects controller and vehicle data for further analysis.
