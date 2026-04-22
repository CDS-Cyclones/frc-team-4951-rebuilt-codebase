# Autonomous and PathPlanner

Autonomous routines are built with PathPlanner and selected through an AdvantageKit logged dashboard chooser.

## Key Files

- `src/main/deploy/pathplanner/autos`
- `src/main/deploy/pathplanner/paths`
- `src/main/deploy/pathplanner/settings.json`
- `src/main/deploy/pathplanner/navgrid.json`
- `RobotContainer.java`
- `Drive.java`
- command files in `src/main/java/frc/robot/commands`

## How Autos Are Loaded

`Drive.java` configures PathPlanner's `AutoBuilder`. `RobotContainer.java` builds the chooser:

```java
autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
```

PathPlanner auto files placed in `src/main/deploy/pathplanner/autos` can appear in the chooser.

## Named Commands

Named commands connect PathPlanner event markers to robot code.

Example:

```java
NamedCommands.registerCommand("intakeStart", ManipulationCommands.startIntake(intake, kicker));
```

If a PathPlanner auto contains an event named `intakeStart`, PathPlanner can run that command during the path.

Named commands usually include actions like:

- starting an intake
- stopping an intake
- scoring a game piece
- moving a mechanism
- climbing or endgame actions

The exact names are project-specific. Check `RobotContainer.java` for the current list before editing a PathPlanner auto.

Always register named commands before building the auto chooser.

## Adding a New Auto Event

1. Create a command factory in the appropriate command file.
2. Register it in `RobotContainer`.
3. Use the exact same name in PathPlanner.
4. Test in simulation first.
5. Check logs in AdvantageScope.

## Adding a New Path

1. Open PathPlanner.
2. Make sure robot config matches `Constants.DriveConstants.ppConfig`.
3. Create or edit a path under `src/main/deploy/pathplanner/paths`.
4. Create an auto under `src/main/deploy/pathplanner/autos`.
5. Add event markers if needed.
6. Simulate the auto.
7. Test slowly on the real robot.

## ELI5: What PID Is

PID is a way to make the robot fix its own mistake.

Imagine you tell the robot, "drive to this exact spot." The robot checks where it is now, compares that to where it should be, and decides how hard to move.

- `P` means "push based on how far away I am." If the robot is far away, push harder. If it is close, push softer.
- `I` means "fix a small error that has been sitting there for a while." This is useful sometimes, but it can also make robots act weird if tuned badly.
- `D` means "slow down based on how fast the error is changing." It helps stop overshooting and bouncing past the target.

For most FRC drive-to-position code, start by tuning `P`, add a little `D` if the robot overshoots, and avoid `I` unless you understand why you need it.

If PID is too low, the robot moves slowly or never reaches the target. If PID is too high, the robot shakes, overshoots, or drives aggressively past the target.

## Drive To Pose

For robot-positioning autos, start by copying the existing `DriveCommands.driveToPose(...)` pattern. This is the version that already works with this codebase's drive subsystem, pose estimator, vision subsystem, and `Constants.DriveConstants.FieldPose` targets.

Use it like the current controller bindings do in `RobotContainer.java`:

```java
DriveCommands.driveToPose(
    drive,
    vision,
    () -> Constants.DriveConstants.FieldPose.leftScore);
```

Do not rewrite drive-to-pose from scratch unless there is a clear reason. Copy the working pattern, create a new `FieldPose` and tune the PID values, dont be discouraged if it doesnt seem to work, PID is a bitch.

The PID values currently live in `Constants.DriveConstants`:

- `angleController`
- `translationXController`
- `translationYController`
- `driveToPoseMaxLinearSpeedMetersPerSec`
- `driveToPoseMaxAngularSpeedRadPerSec`

Tune these on the actual robot after odometry and vision are correct. A PID setup that works in simulation may still need changes on carpet because the real robot has friction, battery sag, wheel slip, imperfect camera data, and mechanical differences.

## Robot Pose and Alliance

`Robot.autonomousInit()` calls:

```java
robotContainer.resetPoseForAlliance();
```

This resets heading based on alliance. PathPlanner also receives an alliance-color supplier in `Drive.java` so paths can be flipped for the red alliance.

## Debugging Autos

Use AdvantageScope to inspect:

- `Odometry/Robot`
- `Odometry/Trajectory`
- `Odometry/TrajectorySetpoint`
- `SwerveStates/Measured`
- `SwerveStates/Setpoints`
- vision accepted/rejected poses

If the robot drives the wrong direction, check module order, gyro heading, field orientation, and alliance flipping before retuning controllers.

Next: [Simulation, Logging, and Replay](simulation-logging-replay.md)
