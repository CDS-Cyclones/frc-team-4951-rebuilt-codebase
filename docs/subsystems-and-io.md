# Subsystems and IO Layers

Subsystems represent robot mechanisms. They own the mechanism state, expose high-level methods, update logged inputs, and are required by commands.

This project uses the AdvantageKit IO pattern:

- `Subsystem.java`: mechanism behavior.
- `SubsystemIO.java`: interface for inputs and outputs.
- `SubsystemIOSparkMax.java`, `SubsystemIOSparkFlex.java`, etc.: real hardware implementation.
- `SubsystemIOSim.java`: simulation implementation.
- Empty anonymous IO implementation: replay or disabled hardware access.

## Example: Shooter

Shooter files:

- `subsystems/shooter/Shooter.java`
- `subsystems/shooter/ShooterIO.java`
- `subsystems/shooter/ShooterIOSparkMax.java`
- `subsystems/shooter/ShooterIOSim.java`

`ShooterIO` defines logged inputs:

```java
@AutoLog
public static class ShooterIOInputs {
  public double mainAppliedOutput = 0.0;
  public double mainVelocityRPM = 0.0;
}
```

`Shooter` owns behavior:

- `setVelocityRPM(...)`
- `setPower(...)`
- `isAtSpeed()`
- `stop()`
- `isActive()`

Every loop, it updates inputs and logs them:

```java
io.updateInputs(inputs);
Logger.processInputs("Shooter", inputs);
```

## Why This Pattern Matters

The same subsystem can run in multiple environments:

- Real robot: `ShooterIOSparkMax`.
- Simulator: `ShooterIOSim`.
- Replay: `new ShooterIO() {}`.

This means robot logic can be developed with limited robot access. Real hardware code is isolated from mechanism behavior.

## What Belongs in a Subsystem

Put these in a subsystem:

- High-level mechanism actions.
- State checks like `isAtSpeed()` or `hasGamePiece()`.
- Sensor interpretation that belongs to that mechanism.
- Periodic logging.
- Simple safety checks local to that mechanism.

## What Belongs in IO

Put these in an IO implementation:

- Motor controller setup.
- Encoder reads.
- NetworkTables reads for a sensor.
- Current limits, inversion, idle mode, PID controller setup.
- Simulation-specific physics state.

## What Belongs in Commands

Put these in command factories under `commands/`:

- Actions that combine multiple subsystems.
- Driver/operator actions.
- Timed sequences.
- Autonomous event actions.
- Conditional workflows.

Examples:

- `ManipulationCommands` methods for scoring, intaking, and moving game pieces
- `ClimbCommands.climbUp(...)`
- `DriveCommands.driveToPose(...)`

## Adding a New Subsystem

Create a package:

```text
src/main/java/frc/robot/subsystems/example
```

Add:

- `Example.java`
- `ExampleIO.java`
- `ExampleIOSparkMax.java`
- `ExampleIOSim.java` if simulation is useful

Then wire it in `RobotContainer`.

For a step-by-step workflow, see [Adding New Robot Features](adding-new-features.md).

Next: [Drive and Swerve](drive-and-swerve.md)
