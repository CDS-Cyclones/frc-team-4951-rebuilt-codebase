# Vision

The vision system estimates robot pose from AprilTags and sends accepted measurements to the drive pose estimator.

## Key Files

- `subsystems/vision/Vision.java`
- `subsystems/vision/VisionIO.java`
- `subsystems/vision/VisionIOLimelight.java`
- `subsystems/vision/VisionIOPhotonVision.java`
- `subsystems/vision/VisionIOPhotonVisionSim.java`
- `Constants.VisionConstants`
- `LimelightHelpers.java`

## Real Robot Vision

Real robot mode uses `VisionIOLimelight`:

```java
new VisionIOLimelight(Constants.VisionConstants.camera0Name, drive::getRotation)
```

The camera names in `Constants.VisionConstants` must match the Limelight NetworkTables names:

```java
public static String camera0Name = "limelight";
public static String camera1Name = "limelight-rear";
```

## Limelight Source Guidance

For Limelight-specific code, use the official Limelight documentation and templates as the source of truth. This project already includes `LimelightHelpers.java`, which is commonly copied from Limelight's published helper file.

Use Limelight templates/docs for:

- NetworkTables key names.
- MegaTag and MegaTag2 output formats.
- Camera pose configuration.
- `robot_orientation_set` usage.
- Latency and timestamp recommendations.
- Limelight web UI setup.

Do not guess Limelight array indexes. Confirm them against Limelight documentation before changing parsing code.

## Current Limelight IO Behavior

`VisionIOLimelight` reads:

- `tx`
- `ty`
- `tl`
- `botpose_wpiblue`
- `botpose_orb_wpiblue`

It publishes:

- `robot_orientation_set`

MegaTag2 needs the robot's current heading. That is why `VisionIOLimelight` receives `drive::getRotation`.

## Vision Filtering

`Vision.java` rejects observations when:

- no tags are visible
- a single-tag observation has too much ambiguity
- Z error is unrealistic
- the pose is outside field boundaries

Accepted observations are converted into standard deviations based on:

- average tag distance
- number of visible tags
- observation type
- per-camera trust multiplier

Then they are passed to the drive pose estimator.

## Simulation Vision

Simulation mode uses `VisionIOPhotonVisionSim`, not Limelight. This lets simulated AprilTag observations be generated from the simulated robot pose.

Robot-to-camera transforms for simulation are stored in `Constants.VisionConstants`:

- `botToCamTransformSim`
- `botToCamTransformSimRear`

Real Limelight transforms are configured in the Limelight web UI. The constants `robotToCamera0` and `robotToCamera1` are not used by the Limelight IO implementation.

## Things to Tune Carefully

- Camera names.
- Camera mounting position and rotation.
- Limelight web UI robot-to-camera transform.
- AprilTag field layout.
- `maxAmbiguity`.
- `maxZError`.
- standard deviation baselines.
- per-camera standard deviation factors.

Bad camera transforms or timestamps can make pose estimation worse than no vision.

Next: [Autonomous and PathPlanner](autonomous-and-pathplanner.md)

