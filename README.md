# FRC Team 4951 Robot Code

This repository contains Team 4951's Java/WPILib robot code. It is built around the AdvantageKit SparkSwerve template style, uses AdvantageKit for logging and replay, and is structured to run on the real robot, in simulation, or from replayed logs.

The documentation is written for future programmers who need to adapt this code for a new FRC season.

## Start Here

1. [Introduction](docs/introduction.md)
2. [Project Overview](docs/project-overview.md)
3. [Coding From the AdvantageKit SparkSwerve Template](docs/advantagekit-sparkswerve-template.md)
4. [RobotContainer Guide](docs/robotcontainer.md)
5. [Subsystems and IO Layers](docs/subsystems-and-io.md)
6. [Drive and Swerve](docs/drive-and-swerve.md)
7. [Vision](docs/vision.md)
8. [Autonomous and PathPlanner](docs/autonomous-and-pathplanner.md)
9. [WPILib Command Syntax](docs/wpilib-command-syntax.md)
10. [Simulation, Logging, and Replay](docs/simulation-logging-replay.md)
11. [Adding New Robot Features](docs/adding-new-features.md)
12. [External Code and Templates](docs/external-code-and-templates.md)

## New Season Bring-Up

At the start of each season, do the software and control-system updates before spending time debugging robot code:

- Install the current season's WPILib release.
- Install or update vendor libraries in `vendordeps`.
- Image/update the roboRIO with the current season firmware using the roboRIO Imaging Tool.
- Update the robot radio with the current FRC radio configuration utility.
- Update Driver Station software on the driver laptop.
- Confirm the team number, radio SSID, roboRIO network connection, and firewall settings.
- Rebuild the project and deploy a simple known-good program before testing full robot code.

If the roboRIO image, radio configuration, Driver Station, WPILib, or vendor libraries are from different seasons, basic robot communication and deploys can fail even when the Java code is correct.

## Important Files

- `src/main/java/frc/robot/Robot.java`: robot lifecycle, AdvantageKit setup, scheduler, simulation hooks.
- `src/main/java/frc/robot/RobotContainer.java`: creates subsystems, selects real/sim/replay IO, registers autos, binds controllers.
- `src/main/java/frc/robot/Constants.java`: robot-wide constants, CAN IDs, tuning values, field poses, camera names, simulation config.
- `src/main/java/frc/robot/subsystems`: subsystem code.
- `src/main/java/frc/robot/commands`: reusable robot actions.
- `src/main/deploy/pathplanner`: PathPlanner autos, paths, settings, and navgrid.
- `vendordeps`: installed vendor libraries.

## Mental Model

The code is split into three main layers:

- `Robot.java` runs the robot program lifecycle.
- `RobotContainer.java` wires the robot together.
- Subsystems and commands contain the robot behavior.

Most hardware access should go through an `IO` interface. That lets the same subsystem run on real hardware, in simulation, or in replay without rewriting the subsystem logic.
