# Perabots-Webots-Simulation
Webots simulation for the elimination round of Perabots Challenge

This project is a Webots simulation of an autonomous obstacle-avoiding wall-following robot. The robot can detect walls and obstacles using distance sensors and adjusts its speed and motion accordingly. The project features adaptive speed control, Bezier smoothing for motor commands, and a reactive wall-following behavior.

# Key Features

## Obstacle Avoidance & Wall-Following
- Uses 6 proximity sensors (ps1–ps6) to detect walls and obstacles.
- Switches between searching, wall-following, and cornering behaviors based on sensor input.

## Adaptive Speed Control
- Implements corner and straight path performance tracking.
- Adjusts motor speed automatically based on recent sensor readings.
- Ensures smooth navigation in both turns and straight paths.

## Bezier Curve Smoothing for Motors
- Smooths wheel speed changes using a Bezier-based linear interpolation.
- Prevents jerky movement and ensures stable navigation.

## PID-Inspired Adaptive Motor Control
- Reactively adjusts left and right wheel velocities for corners, straights, and escape maneuvers.
- Optimizes speed while preventing collisions and maintaining wall-following accuracy.

## Simulation in Webots
- Fully simulated robot environment with .wbt world files.
- Includes configurable robot sensors and motors for testing behaviors before real-world implementation.
