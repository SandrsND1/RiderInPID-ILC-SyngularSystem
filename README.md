# RiderInPID-ILC Syngular System

Unity-based vehicle simulation implementing PID control and Iterative Learning Control (ILC) for autonomous track following.

The project combines a kinematic bicycle vehicle model, PID feedback control, cross-track error correction and iterative learning between laps.

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
PID Controller

The controller calculates the steering command using proportional, integral and derivative terms.

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

During the first lap, the controller records the cross-track error.

On subsequent laps, the stored error is used to update the steering correction:

ILC[k] = ILC[k] - learningRate * previousCTE[k]

The correction is limited to prevent unstable steering commands.

Default parameters:

learningRate = 0.02
maxILC = 0.16 rad

The ILC learning process starts after the first completed lap.

Vehicle Model

The vehicle uses a simplified kinematic bicycle model.

State variables:

x      - X position
y      - Y position
theta  - vehicle heading
delta  - steering angle
v      - vehicle velocity

Vehicle dynamics:

dx     = v * cos(theta)
dy     = v * sin(theta)
dtheta = (v / L) * tan(delta)

Default wheelbase:

L = 2.5 m

The vehicle model also limits steering angle and velocity and normalizes the heading angle to [-π, π].

Track Following

TrackFollower calculates the vehicle's position relative to the track.

It provides:

nearest track point
lookahead point
cross-track error
target heading
Cross-Track Error

The vehicle position is projected onto the nearest track segment.

The distance from the vehicle to the projected point represents the tracking error.

The sign of the error is determined using the track normal.

CTE > 0  -> vehicle is on one side of the track
CTE < 0  -> vehicle is on the opposite side
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

After the final lap, the vehicle is stopped and the race statistics are printed to the Unity console.

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
cross-track error
ILC correction
ILC index
nearest track index

The recorded data can be used to evaluate controller performance and compare tracking behavior across laps.

Responsibilities:

PID control
ILC learning
steering control
speed control
lap tracking
controller state management
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
target heading
TrackPath

Stores the track geometry and provides:

nearest point search
track direction
track visualization
RaceManager

Controls the race and multi-lap simulation.

DataRecorder

Collects controller and vehicle data for further analysis.

Main Parameters
Parameter	Default	Description
kp	3.5	Proportional gain
ki	0.08	Integral gain
kd	0.9	Derivative gain
maxIntegral	0.4	Integral anti-windup limit
derivativeFilterK	0.7	Derivative filtering
learningRate	0.02	ILC learning rate
maxILC	0.16	Maximum ILC correction
maxSteering	0.75	Maximum steering angle
steeringRate	6.0	Maximum steering change
targetSpeed	4.0	Target speed
lookaheadDistance	6.0	Lookahead distance
acceleration	3.0	Acceleration
Requirements
Unity
C#
Unity Input System
Unity URP

The exact Unity version can be found in:

ProjectSettings/ProjectVersion.txt
Running the Project
Clone the repository.
Open the project using Unity Hub.
Open:
Assets/Scenes/SampleScene.unity
Press Play.

The vehicle should follow the predefined track using the PID-ILC controller.

Debugging

The controller provides runtime information including:

Current Lap
ILC Index
ILC Points
Distance
Velocity
Cross-Track Error
Steering
Maximum ILC Correction

Track visualization can also display the lookahead point and direction.

Purpose

The project was developed as a vehicle control simulation for studying trajectory tracking and iterative learning control.

The main idea is to combine classical PID feedback with ILC-based correction learned from repeated traversal of the same trajectory.

Future Improvements
Adaptive lookahead based on vehicle speed
Curvature-based speed control
Improved lap detection
Advanced ILC learning laws
ILC filtering and regularization
Automatic controller parameter optimization
PID vs PID-ILC performance comparison
Trajectory error visualization
Python/MATLAB analysis pipeline
