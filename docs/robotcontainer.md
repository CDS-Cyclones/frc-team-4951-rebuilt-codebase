# RobotContainer Guide

`RobotContainer.java` is the wiring file for the robot. It creates subsystems, connects them to hardware or simulation, registers autonomous commands, and maps controller buttons to commands.

## What RobotContainer Owns

`RobotContainer` owns:

- Subsystem instances.
- Controller objects.
- The autonomous chooser.
- Real/sim/replay IO selection.
- Button bindings.
- Simulation field reset and display helpers.
- High-level logging that combines multiple subsystem states.

It should not contain detailed mechanism logic. If a behavior grows beyond simple wiring, move it into `commands/` or the relevant subsystem.

## Mode-Based Construction

The constructor switches on `Constants.currentMode`.

In `REAL`, it creates real hardware IO:

- `GyroIOPigeon2`
- `ModuleIOSpark`
- `VisionIOLimelight`
- Spark-based mechanism IO classes

In `SIM`, it creates simulated IO:

- `SwerveDriveSimulation`
- `GyroIOSim`
- `ModuleIOSim`
- `VisionIOPhotonVisionSim`
- mechanism sim IO classes

In `REPLAY`, it creates empty IO implementations:

```java
new ShooterIO() {}
```

Replay mode should avoid talking to hardware. It reads from logs instead.

## Subsystem Creation Pattern

Subsystems receive IO implementations through constructors:

```java
shooter = new Shooter(new ShooterIOSparkMax());
```

The subsystem does not need to know whether the IO talks to a Spark, a simulator, or nothing. This is the main AdvantageKit pattern used throughout the project.

## Autonomous Setup

RobotContainer registers PathPlanner named commands:

```java
NamedCommands.registerCommand("intakeStart", ManipulationCommands.startIntake(intake, kicker));
```

These names can be used as event markers inside PathPlanner autos. If a PathPlanner auto references a named command, that command must be registered before the auto chooser is built.

Then the chooser is created:

```java
autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
```

Extra test or characterization autos can be added using `autoChooser.addOption(...)`.

## Controller Bindings

`configureButtonBindings()` maps controller buttons to commands.

Current controller roles:

- Driver controller on port `0`.
- Operator controller on port `1`.
- Test controller on port `2`.

The drive subsystem gets a default command:

```java
drive.setDefaultCommand(normalDriveCommand);
```

A default command runs whenever no other command is using that subsystem.

Button methods use common command trigger patterns:

- `onTrue(...)`: start once when pressed.
- `whileTrue(...)`: run while held.
- `toggleOnTrue(...)`: toggle on and off each press.

For a deeper explanation of command syntax, lambdas like `() ->`, and trigger bindings, see [WPILib Command Syntax](wpilib-command-syntax.md).

## Simulation Helpers

`resetSimulationField()` resets the simulated robot and game pieces when disabled.

`displaySimFieldToAdvantageScope()` logs the simulated robot pose so AdvantageScope can display it.

## Where New Code Should Go

Add a new subsystem field near the other subsystem fields. Instantiate it inside all three mode branches:

- Real IO in `REAL`.
- Sim IO or blank IO in `SIM`.
- Blank IO in `REPLAY`.

Add commands in `commands/`, not directly in controller bindings, if the action is more than one or two lines.

Next: [Subsystems and IO Layers](subsystems-and-io.md)
