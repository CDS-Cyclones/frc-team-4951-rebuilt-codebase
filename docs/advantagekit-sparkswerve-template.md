# Coding From the AdvantageKit SparkSwerve Template

This codebase is built around the AdvantageKit SparkSwerve template style from Mechanical Advantage. The drive subsystem, IO structure, logging style, and replay-friendly organization come from that pattern.

## What the Template Gives You

The AdvantageKit SparkSwerve template usually provides:

- A swerve `Drive` subsystem.
- `Module`, `ModuleIO`, and hardware/sim module implementations.
- `GyroIO` and gyro implementations.
- Spark odometry threading.
- AdvantageKit input logging using `@AutoLog`.
- Real/sim/replay mode separation.
- PathPlanner `AutoBuilder` setup.
- Pose estimation and vision-measurement hooks.

In this project, those ideas appear in:

- `src/main/java/frc/robot/subsystems/drive/Drive.java`
- `src/main/java/frc/robot/subsystems/drive/Module.java`
- `src/main/java/frc/robot/subsystems/drive/ModuleIO.java`
- `src/main/java/frc/robot/subsystems/drive/ModuleIOSpark.java`
- `src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java`
- `src/main/java/frc/robot/subsystems/drive/GyroIO.java`
- `src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java`
- `src/main/java/frc/robot/subsystems/drive/GyroIOSim.java`
- `src/main/java/frc/robot/subsystems/drive/SparkOdometryThread.java`

## How to Adapt the Template

1. Update robot dimensions in `Constants.DriveConstants`.
2. Update CAN IDs for the gyro and all four swerve modules.
3. Update module zero rotations after physically zeroing the modules.
4. Confirm wheel radius, gear ratios, current limits, and motor types.
5. Confirm the CAN bus name.
6. Update PathPlanner robot config values: mass, moment of inertia, wheel coefficient of friction, module positions.
7. Run drive characterization and tune feedforward/PID values.
8. Validate odometry in AdvantageScope.
9. Add vision measurements only after camera poses and timestamps are correct.

## What to Keep

Keep the core IO separation:

- Subsystem classes should make decisions and expose high-level robot actions.
- IO classes should talk to motors, encoders, sensors, NetworkTables, or simulation.
- Constants should hold hardware IDs and tunable values.

This separation is what makes AdvantageKit replay and simulation useful. If subsystem logic directly reads motor controllers, replay becomes much less valuable.

## What to Replace

Replace anything tied to the original template robot:

- CAN IDs.
- Module offsets.
- Drivetrain dimensions.
- Camera names and camera transforms.
- Robot mass and moment of inertia.
- Field-specific AprilTag layout.
- Autonomous paths.
- Mechanism subsystems.

## Generated AdvantageKit Classes

Files with names like `ShooterIOInputsAutoLogged` are generated from `@AutoLog` input classes. You do not manually create them. If the generated class is missing, build the project:

```bash
./gradlew build
```

## Good Rule

If code is hardware-specific, put it in an IO implementation. If code describes what the robot should do, put it in the subsystem or command layer.

Next: [Subsystems and IO Layers](subsystems-and-io.md)

