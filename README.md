# FRC Team 4951 Competition Robot

This repository contains Team 4951's command-based robot code for the 2026 competition season. It is built on WPILib, uses AdvantageKit for logging, and is structured to run in real robot, simulation, or replay mode.

## What Is In Here

The robot code is organized around these major subsystems:

- Swerve drive
- Vision
- Intake
- Intake arm
- Intake arm kicker
- Kicker
- Hopper
- Shooter
- Climber
- LED

The robot supports:

- Driver and operator controller bindings
- AdvantageKit logging to NT4 and WPILOG
- Simulation support with MapleSim and PhotonVision simulation

## Requirements

- WPILib 2026.2.1
- REV Spark motor controllers
- Limelight cameras for vision on the real robot

Control bindings live in `src/main/java/frc/robot/RobotContainer.java`.

- Driver controller, port 0: Start resets the gyro / pose; left and right bumpers drive around the front modules; left trigger drives to the left score pose; right trigger drives to the right score pose; X drives to the middle score pose; POV up and down manually override the climber; POV left outtakes through the intake and kicker.
- Operator controller, port 1: Left trigger shoots; right trigger passes; POV up and down climb; POV left clears the rung; A toggles intake, kicker, and intake arm kicker; B outtakes intake and kicker; X outtakes intake only; Y toggles the intake arm.

The test controller is reserved for subsystem validation and diagnostic commands.

## Repository Layout

- `src/main/java/frc/robot/` - robot entry points, constants, commands, and subsystems
- `src/main/deploy/` - deploy-time assets such as PathPlanner files
- `vendordeps/` - vendor dependency definitions
- `build.gradle` - GradleRIO build configuration

If you are adding new hardware, follow the existing pattern: define subsystem constants, add an IO interface, add REAL and SIM implementations, then wire the subsystem into `RobotContainer`.
