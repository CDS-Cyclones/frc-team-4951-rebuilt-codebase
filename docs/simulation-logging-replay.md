# Simulation, Logging, and Replay

This project is designed to support development with limited physical robot access. Simulation and logging are not extras; they are part of the normal workflow.

## AdvantageKit Logging

`Robot.java` starts AdvantageKit and configures outputs based on mode.

Real robot:

- Logs to WPILOG.
- Publishes NetworkTables data.

Simulation:

- Publishes NetworkTables data.

Replay:

- Reads an existing WPILOG.
- Writes a replay output log.
- Runs as fast as possible.

## What Gets Logged

Subsystems call:

```java
Logger.processInputs("SubsystemName", inputs);
```

Subsystems and commands can also record custom outputs:

```java
Logger.recordOutput("Shooter/IsActive", active);
```

Useful logged groups include:

- `Drive/Gyro`
- `SwerveStates`
- `SwerveChassisSpeeds`
- `Odometry`
- `Vision`
- `FieldSimulation`
- mechanism-specific logs like `Shooter`, `Intake`, and `Manipulator`

## Simulation Pieces

Simulation mode creates a `SwerveDriveSimulation` and adds it to `SimulatedArena`.

`Robot.simulationPeriodic()` advances simulation and logs:

- simulated robot pose
- simulated game-piece positions, if the current season simulation supports them

`RobotContainer.resetSimulationField()` resets:

- robot pose
- simulated field
- intake simulation state

## Vision Simulation

Simulated vision uses `VisionIOPhotonVisionSim`. It receives the simulated drivetrain pose and produces AprilTag observations for the same `Vision` subsystem used on the real robot.

This is useful because the pose-estimation code can be tested without a Limelight connected.

## Replay Mode

Replay mode should not talk to hardware. `RobotContainer` creates blank IO implementations, and AdvantageKit feeds logged inputs from the replay file.

Use replay to debug:

- subsystem logic
- pose estimation behavior
- command timing
- autonomous decisions
- logging outputs

Replay cannot fix a bad log. If a signal was not logged during the real run, it will not be available later.

## Suggested Workflow With Limited Robot Time

1. Implement subsystem behavior against IO interfaces.
2. Add or update simulation IO.
3. Test commands in simulation.
4. Watch AdvantageScope logs for impossible states.
5. Run on the real robot at low speed or reduced power.
6. Save logs.
7. Replay logs and tune logic before the next robot session.

Next: [Adding New Robot Features](adding-new-features.md)
