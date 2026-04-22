# Project Overview

This project is an FRC robot codebase for CDS Cyclones Team 4951. It is written in Java using WPILib and AdvantageKit, and it is intended to be adapted each season as the robot design and game requirements change.

The codebase uses:

- WPILib robot lifecycle and command framework.
- AdvantageKit logging, replay, and generated input classes.
- REV Spark motor controllers for many mechanisms.
- CTRE Phoenix 6 for the Pigeon2 gyro.
- PathPlanner for autonomous routines.
- Limelight for real robot AprilTag vision.
- PhotonVision simulation IO for simulated vision.
- MapleSim for drivetrain and game-piece simulation.

## Before Coding a New Season

Do these updates first:

- Install the current WPILib version.
- Update vendor dependencies for REV, CTRE, PathPlanner, AdvantageKit, PhotonVision, and any other libraries the robot uses.
- Image the roboRIO with the current season firmware.
- Update and configure the robot radio for the current season.
- Update the Driver Station laptop.
- Confirm the team number is correct in Driver Station and radio configuration.
- Deploy a minimal test program before deploying the full robot project.

These steps are not optional bring-up work. A mismatched roboRIO image, old radio configuration, stale vendor library, or outdated Driver Station can look like a code problem.

## Main Responsibilities

`Robot.java`:

- Starts AdvantageKit logging.
- Selects log outputs for real, sim, or replay mode.
- Runs the command scheduler every loop.
- Starts and resets simulation state.
- Schedules autonomous commands.

`RobotContainer.java`:

- Creates every subsystem.
- Chooses real, simulated, or replay IO implementations.
- Registers PathPlanner named commands.
- Builds the autonomous chooser.
- Defines controller bindings.

`Constants.java`:

- Stores robot dimensions, CAN IDs, camera names, PID values, tunable numbers, and field poses.
- Selects the current runtime mode using `Constants.currentMode`.

`subsystems/`:

- Owns robot mechanisms like drive, intake, shooter, vision, climber, hopper, kicker, and LEDs.
- Uses an IO pattern so subsystem logic is separate from motor-controller details.

`commands/`:

- Contains reusable robot actions that combine one or more subsystems.
- Examples include driving, shooting, intaking, climbing, and testing actions.

## Runtime Modes

The project uses three modes:

- `REAL`: runs on the roboRIO with real motor controllers, sensors, and Limelights.
- `SIM`: runs physics simulation using simulated drive, mechanisms, and simulated vision.
- `REPLAY`: replays an AdvantageKit log without talking to real hardware.

The mode is selected in `Constants.java`:

```java
public static final Mode simMode = Mode.SIM;
public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
```

When running on a real robot, `currentMode` is always `REAL`. When running off-robot, change `simMode` to switch between `SIM` and `REPLAY`.

Next: [RobotContainer Guide](robotcontainer.md)
