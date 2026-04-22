# External Code and Templates

This project includes code and patterns that come from external libraries, templates, or vendor documentation. Keep those boundaries clear when modifying the codebase.

## AdvantageKit SparkSwerve Template

Source/pattern owner: Mechanical Advantage.

Used for:

- swerve subsystem architecture
- drive IO pattern
- gyro IO pattern
- module IO pattern
- odometry thread pattern
- AdvantageKit logging/replay structure
- generated input logging through `@AutoLog`

Local files most closely tied to this template:

- `subsystems/drive/*`
- mode switching in `RobotContainer`
- logging setup in `Robot`
- IO interfaces across subsystems

When updating template-derived drive code, compare against the current Mechanical Advantage template rather than guessing.

## WPILib

Source/pattern owner: WPILib.

Used for:

- robot lifecycle
- command scheduling
- controllers and triggers
- geometry, kinematics, pose estimation
- SysId
- units
- alerts
- NetworkTables

Use WPILib docs for lifecycle behavior, command semantics, and pose-estimator APIs.

## REVLib

Source/pattern owner: REV Robotics.

Used for:

- Spark motor controller APIs
- Spark configuration
- encoder access
- current limits
- closed-loop control

Use REV documentation for Spark Max, Spark Flex, NEO, and NEO Vortex behavior.

## Phoenix 6

Source/pattern owner: CTRE.

Used for:

- Pigeon2 gyro IO

Use CTRE Phoenix 6 docs for Pigeon2 configuration, status signals, and CAN bus behavior.

## Limelight

Source/pattern owner: Limelight.

Used for:

- AprilTag pose estimation on the real robot
- MegaTag and MegaTag2 NetworkTables data
- `robot_orientation_set`
- `LimelightHelpers.java`

Use official Limelight docs/templates for NetworkTables keys, array formats, timestamps, and camera setup. The Limelight web UI is where real camera-to-robot transforms are configured for this codebase.

## PhotonVision

Source/pattern owner: PhotonVision.

Used for:

- simulated vision IO
- AprilTag camera simulation

Use PhotonVision docs for simulated camera properties and target-layout behavior.

## PathPlanner

Source/pattern owner: PathPlanner.

Used for:

- auto files
- path files
- `AutoBuilder`
- named commands
- pathfinding
- trajectory logging callbacks

Use PathPlanner docs when changing auto config, event markers, path constraints, and alliance flipping.

## MapleSim

Source/pattern owner: Iron Maple.

Used for:

- `SwerveDriveSimulation`
- `SimulatedArena`
- season-specific game-piece simulation
- drivetrain physics config

Use MapleSim docs/examples for simulation-specific behavior, field reset, game-piece simulation, and drivetrain model setup.

## URCL

Source/pattern owner: Littleton Robotics/URCL.

Used for:

- REV device logging through AdvantageKit.

`Robot.java` starts URCL and disables REVLib's built-in auto logging.

## Local Project Code

Team-specific code includes:

- mechanism subsystems outside the base drive template
- mechanism commands
- robot constants
- controller mappings
- autonomous command choices
- field poses and game-specific logic
- simulation behavior for custom mechanisms

When modifying local code, prefer the patterns already used in this repository.

Back to [Robot Code Documentation](../README.md)
