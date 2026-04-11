// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private double latestAverageTagDistance = -1.0;

  /** Index of the front camera, used for hub distance reporting. */
  private static final int FRONT_CAMERA_INDEX = 0;

  /** Distance to hub tags as seen by the front camera only, or -1 if not visible. */
  private double frontCameraHubTagDistance = -1.0;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /** Returns the tag IDs currently visible to the specified camera. */
  public int[] getVisibleTagIds(int cameraIndex) {
    return inputs[cameraIndex].tagIds;
  }

  /** Returns the total number of unique AprilTags visible across all cameras. */
  public int getTotalVisibleTagCount() {
    java.util.Set<Integer> seen = new java.util.HashSet<>();
    for (var input : inputs) {
      for (int id : input.tagIds) seen.add(id);
    }
    return seen.size();
  }

  /** Returns true if any of the specified tag IDs are currently visible across all cameras. */
  public boolean isAnyTagVisible(int[] tagIds) {
    for (var input : inputs) {
      for (int visibleId : input.tagIds) {
        for (int targetId : tagIds) {
          if (visibleId == targetId) return true;
        }
      }
    }
    return false;
  }

  /**
   * Returns true if the front camera can currently see any of the specified tag IDs. Use this
   * instead of {@link #isAnyTagVisible} when you need to ensure only the front camera is considered
   * (e.g. for distance-based indicators that should not be confused by rear-camera sightings).
   */
  public boolean isFrontCameraTagVisible(int[] tagIds) {
    if (FRONT_CAMERA_INDEX >= inputs.length) return false;
    for (int visibleId : inputs[FRONT_CAMERA_INDEX].tagIds) {
      for (int targetId : tagIds) {
        if (visibleId == targetId) return true;
      }
    }
    return false;
  }

  /**
   * Returns the average tag distance from the latest accepted pose observation, or -1 if no
   * observations are available.
   */
  public double getLatestAverageTagDistance() {
    return latestAverageTagDistance;
  }

  /**
   * Returns the average hub-tag distance as seen by the front camera only, or -1 if the front
   * camera does not currently see any alliance hub tags. Use this for driver-facing indicators that
   * should not be affected by rear-camera observations.
   */
  public double getFrontCameraHubTagDistance() {
    return frontCameraHubTagDistance;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    double closestAcceptedDistance = -1.0;

    // Track front-camera hub distance separately for driver indicators
    int[] allianceHubTagIds =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red
            ? Constants.VisionConstants.DISTANCE_TRACKING_RED_TAG_IDS
            : Constants.VisionConstants.DISTANCE_TRACKING_BLUE_TAG_IDS;
    double frontHubDistance = -1.0;

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = Constants.VisionConstants.aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Check if this is the front camera and it sees alliance hub tags
      boolean isFrontCameraWithHubTags = false;
      if (cameraIndex == FRONT_CAMERA_INDEX) {
        for (int visibleId : inputs[cameraIndex].tagIds) {
          for (int hubId : allianceHubTagIds) {
            if (visibleId == hubId) {
              isFrontCameraWithHubTags = true;
              break;
            }
          }
          if (isFrontCameraWithHubTags) break;
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > Constants.VisionConstants.maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > Constants.VisionConstants.maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX()
                    > Constants.VisionConstants.aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY()
                    > Constants.VisionConstants.aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = Constants.VisionConstants.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = Constants.VisionConstants.angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= Constants.VisionConstants.linearStdDevMegatag2Factor;
          angularStdDev *= Constants.VisionConstants.angularStdDevMegatag2Factor;
        }
        if (cameraIndex < Constants.VisionConstants.cameraStdDevFactors.length) {
          linearStdDev *= Constants.VisionConstants.cameraStdDevFactors[cameraIndex];
          angularStdDev *= Constants.VisionConstants.cameraStdDevFactors[cameraIndex];
        }

        // // Send vision observation
        // System.out.println(
        //     "Vision -> PoseEstimator camera="
        //         + cameraIndex
        //         + " pose="
        //         + observation.pose().toPose2d()
        //         + " timestamp="
        //         + observation.timestamp()
        //         + " tags="
        //         + observation.tagCount()
        //         + " avgTagDist="
        //         + observation.averageTagDistance()
        //         + " type="
        //         + observation.type()
        //         + " stdDevs=["
        //         + linearStdDev
        //         + ", "
        //         + linearStdDev
        //         + ", "
        //         + angularStdDev
        //         + "]");

        // Track closest accepted tag distance
        if (closestAcceptedDistance < 0
            || observation.averageTagDistance() < closestAcceptedDistance) {
          closestAcceptedDistance = observation.averageTagDistance();
        }

        // Track front-camera hub distance for driver indicators
        if (isFrontCameraWithHubTags
            && (frontHubDistance < 0 || observation.averageTagDistance() < frontHubDistance)) {
          frontHubDistance = observation.averageTagDistance();
        }

        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    latestAverageTagDistance = closestAcceptedDistance;
    frontCameraHubTagDistance = frontHubDistance;

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
