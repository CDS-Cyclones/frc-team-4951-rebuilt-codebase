# Drive and Swerve

The drive subsystem is based on the AdvantageKit SparkSwerve template. It handles swerve module control, odometry, pose estimation, PathPlanner integration, SysId routines, and vision measurement intake.

## Key Files

- `subsystems/drive/Drive.java`
- `subsystems/drive/Module.java`
- `subsystems/drive/ModuleIO.java`
- `subsystems/drive/ModuleIOSpark.java`
- `subsystems/drive/ModuleIOSim.java`
- `subsystems/drive/GyroIO.java`
- `subsystems/drive/GyroIOPigeon2.java`
- `subsystems/drive/GyroIOSim.java`
- `subsystems/drive/SparkOdometryThread.java`

## Drive Constants

Drive constants live in `Constants.DriveConstants`.

Important values:

- `maxSpeedMetersPerSec`
- `trackWidth`
- `wheelBase`
- `driveBaseRadius`
- module CAN IDs
- module zero rotations
- wheel radius
- drive and turn gear reductions
- PID and feedforward values
- PathPlanner `RobotConfig`
- MapleSim drivetrain config

These values must match the real robot. Incorrect dimensions, offsets, or gear ratios will cause odometry and autonomous path following problems.

## Module Order

The modules are created in this order:

```text
0: front left
1: front right
2: back left
3: back right
```

Keep this order consistent anywhere module arrays are used.

## Pose Estimation

`Drive` owns a `SwerveDrivePoseEstimator`. It updates odometry from:

- gyro yaw
- swerve module positions
- timestamped odometry samples
- accepted vision measurements

Vision data enters through:

```java
drive.addVisionMeasurement(...)
```

The `Vision` subsystem calls this through a `VisionConsumer` passed in from `RobotContainer`.

## PathPlanner Setup

`Drive` configures `AutoBuilder` in its constructor:

```java
AutoBuilder.configure(
    this::getPose,
    this::setPose,
    this::getChassisSpeeds,
    speeds -> runVelocity(speeds),
    ...
);
```

That lets PathPlanner autos control the drive using the same pose estimator and velocity-control path used elsewhere, keep in mind PathPlanner requires you to have many PID values set right.s

## Driver Control

The default drive command is set in `RobotContainer` using `DriveCommands.joystickDrive(...)`.

Holding bumpers changes the center of rotation to pivot around a front module. Triggers can run drive-to-pose behavior using vision-assisted field poses.

## Characterization

`Drive` exposes SysId commands and helper commands for:

- wheel radius characterization
- feedforward characterization
- quasistatic tests
- dynamic tests

Only run characterization when the robot is safe, lifted or placed correctly, and the team knows what motion to expect.

## Before Driving a New Robot

Check:

- CAN IDs are correct.
- Pigeon ID and CAN bus are correct.
- Module offsets are calibrated.
- Drive and turn inversions are correct.
- Wheels spin in the expected directions.
- Forward on joystick means forward on the field/robot as intended.
- AdvantageScope shows a sane robot pose.
- PathPlanner robot config matches the robot.

Next: [Vision](vision.md)

