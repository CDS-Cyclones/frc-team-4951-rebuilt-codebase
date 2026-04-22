# Introduction

Hi. This guide is for the next programmers who need to make a new robot project from the AdvantageKit SparkSwerve template and then grow it into a full competition codebase.

The goal is not to memorize every file. The goal is to understand the workflow:

1. Start from a clean AdvantageKit SparkSwerve template.
2. Update the control system for the new season.
3. Configure the drivetrain until it is reliable.
4. Add subsystems one at a time.
5. Test with simulation and logs before spending robot time.
6. Keep hardware code separated from robot logic.

## Step 1: Install Season Software

Before touching code, install the current FRC season tools, Lots of it comes from National Instruments, others come from the WPILib installer:

- WPILib VS Code.
- FRC Driver Station.
- roboRIO Imaging Tool.
- FRC radio configuration utility.
- REV Hardware Client.
- CTRE Tuner X, if using CTRE devices. (I.E Pigeon2/Canivore)
- AdvantageScope.
- PathPlanner.
- Git.

Then update the robot control system:

- Image/update the roboRIO with the current season firmware.
- Update and configure the robot radio.
- Confirm the Driver Station laptop is on the current season software.
- Confirm the team number is correct everywhere.

Do this first. Old roboRIO firmware, old radio firmware/config, old Driver Station software, or stale vendor libraries can waste hours because the robot may fail to connect or deploy even if the Java code is fine.

## Step 2: Get the Spark Swerve Template

AdvantageKit's official setup flow is to download the Spark Swerve template project from the AdvantageKit GitHub release and open it in WPILib VS Code.

Useful links:

- AdvantageKit Spark Swerve docs (you will be using this if its the same drivetrain as 2026/2025/2024): <https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template/>
- AdvantageKit GitHub repository: <https://github.com/Mechanical-Advantage/AdvantageKit>
- AdvantageKit releases: <https://github.com/Mechanical-Advantage/AdvantageKit/releases>



## Step 3: Make It Your Robot Project

After opening the template project:

1. Set the team number from WPILib VS Code using `WPILib: Set Team Number`.
2. Rename the project/repository if needed.
3. Commit the clean template before making robot-specific edits.
4. Install required vendor libraries for the season.
5. Build once before changing code:

```bash
./gradlew build
```

This gives you a known starting point. If the clean template does not build, fix the install or dependency problem before adding team code.

## Step 4: Configure the Swerve Drive

Update drivetrain constants before writing scoring code, lots of this information you will set yourself, CAN id's are set through REV Hardware Client 2, for REV Robotics devices (i.e NEO's), and CTRE Tuner X for CTRE Devices (I.E, Pigeon2, CANivore, which creates a new CAN bus for auxiliary devices (i.e, gyro, but motors cannot be on it)).

You will need:

- drive motor CAN IDs
- turn motor CAN IDs
- gyro CAN ID
- CAN bus name (rio for the rio bus, and can be set differently for canivore)
- wheel radius
- drive gear reduction
- turn gear reduction
- track width
- wheelbase
- module order
- module zero rotations
- current limits
- motor inversions

Deploy and verify the drive carefully:

1. Confirm every CAN device appears in REV Hardware Client or CTRE Tuner X.
2. Confirm there are no Driver Station errors.
3. Open AdvantageScope.
4. Rotate each module by hand and verify the logged angle changes correctly.
5. Rotate each wheel by hand and verify the logged drive position changes correctly.
6. Set module zero rotations.
7. Drive slowly on blocks first.
8. Drive slowly on carpet.
9. Only then tune higher-speed behavior.

Do not build autos until odometry is trustworthy.

## Step 5: Add Team Subsystems

Add each mechanism using the IO pattern:

- `Subsystem.java` for behavior.
- `SubsystemIO.java` for logged inputs and hardware outputs.
- `SubsystemIOSparkMax.java`, `SubsystemIOSparkFlex.java`, or another real hardware implementation.
- `SubsystemIOSim.java` when simulation is useful.

Wire each subsystem into `RobotContainer` in all modes:

- real hardware IO for `REAL`
- sim IO for `SIM`
- empty IO for `REPLAY`

See [Subsystems and IO Layers](subsystems-and-io.md) for the detailed pattern.

## Step 6: Add Controls and Autos

Controller bindings belong in `RobotContainer`.

Reusable actions belong in `commands/`.

PathPlanner event actions must be registered with `NamedCommands` before the auto chooser is built.

Build simple commands first. Test one mechanism at a time. Then combine mechanisms into scoring, intake, climbing, or endgame routines.

## Step 7: Use Simulation and Logs

When robot access is limited, use:

- WPILib simulation.
- AdvantageScope.
- AdvantageKit logs.
- AdvantageKit replay.
- PathPlanner simulation checks.

Logs are how future programmers understand what the robot actually did. If a value matters for debugging, log it.

## Step 8: Keep Notes Updated

When you change something important, update the docs. Future programmers should not have to reverse-engineer:

- CAN IDs.
- controller bindings.
- camera names.
- module offsets.
- autonomous command names.
- subsystem safety rules.
- setup steps that are easy to forget.

Next: [Project Overview](project-overview.md)
