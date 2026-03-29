package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;

/**
 * Backs the robot up to a tower post by detecting an AprilTag on the post. The robot first
 * identifies which tag it can see, then computes a goal pose that places the rear bumper centered
 * on the post, and drives there facing away from the tag.
 */
public class BackupToTagCommand extends Command {
  private static final double TRANSLATION_KP = 2.0;
  private static final double TRANSLATION_KD = 0.1;
  private static final double MAX_LINEAR_VELOCITY = 2.0; // m/s, slower for precision
  private static final double MAX_LINEAR_ACCELERATION = 2.0; // m/s^2

  private static final double HEADING_KP = 5.0;
  private static final double HEADING_KD = 0.4;
  private static final double MAX_ANGULAR_VELOCITY = 6.0; // rad/s
  private static final double MAX_ANGULAR_ACCELERATION = 15.0; // rad/s^2

  private static final double POSITION_TOLERANCE = 0.03; // meters
  private static final double HEADING_TOLERANCE = Math.toRadians(2.0); // radians

  // Distance from robot center to rear bumper edge (half the 0.9m robot)
  private static final double ROBOT_CENTER_TO_REAR = 0.45;
  // Small gap so we don't slam into the post
  private static final double STANDOFF = 0.05; // meters

  private final Drive drive;
  private final Vision vision;
  private final int cameraIndex;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          TRANSLATION_KP,
          0.0,
          TRANSLATION_KD,
          new TrapezoidProfile.Constraints(MAX_LINEAR_VELOCITY, MAX_LINEAR_ACCELERATION));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          TRANSLATION_KP,
          0.0,
          TRANSLATION_KD,
          new TrapezoidProfile.Constraints(MAX_LINEAR_VELOCITY, MAX_LINEAR_ACCELERATION));
  private final ProfiledPIDController headingController =
      new ProfiledPIDController(
          HEADING_KP,
          0.0,
          HEADING_KD,
          new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION));

  private Pose2d targetPose;
  private boolean valid = false;

  /**
   * Creates a new BackupToTagCommand.
   *
   * @param drive The drive subsystem.
   * @param vision The vision subsystem, used to detect which tag is visible.
   * @param cameraIndex The camera index to check for visible tags (typically 0).
   */
  public BackupToTagCommand(Drive drive, Vision vision, int cameraIndex) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    valid = false;

    // Find the first visible tag
    int[] visibleTags = vision.getVisibleTagIds(cameraIndex);
    if (visibleTags.length == 0) {
      DriverStation.reportWarning("BackupToTagCommand: No AprilTags visible.", false);
      return;
    }

    // Use the first visible tag
    int tagId = visibleTags[0];
    Optional<Pose3d> tagPoseOptional = Constants.VisionConstants.aprilTagLayout.getTagPose(tagId);
    if (tagPoseOptional.isEmpty()) {
      DriverStation.reportWarning("BackupToTagCommand: Tag " + tagId + " not in layout.", false);
      return;
    }

    Pose3d tagPose3d = tagPoseOptional.get();
    Translation2d tagPosition = tagPose3d.getTranslation().toTranslation2d();
    // The tag faces outward from the post. The robot needs to be in front of the tag
    // with its BACK toward the tag.
    Rotation2d tagFacing = tagPose3d.getRotation().toRotation2d();

    // Robot center position: offset from the tag along its facing direction
    // by the distance from center to rear bumper plus a small standoff
    double centerOffset = ROBOT_CENTER_TO_REAR + STANDOFF;
    Translation2d goalPosition = tagPosition.plus(new Translation2d(centerOffset, tagFacing));

    // Robot faces AWAY from the tag (same direction the tag faces)
    Rotation2d goalHeading = tagFacing;

    targetPose = new Pose2d(goalPosition, goalHeading);

    Pose2d currentPose = drive.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    headingController.reset(currentPose.getRotation().getRadians());

    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    headingController.setTolerance(HEADING_TOLERANCE);

    valid = true;

    DriverStation.reportWarning(
        "BackupToTagCommand: Backing up to tag " + tagId + " at " + targetPose, false);
  }

  @Override
  public void execute() {
    if (!valid) {
      drive.stop();
      return;
    }

    Pose2d currentPose = drive.getPose();

    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double omega =
        headingController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    double linearSpeed = Math.hypot(xSpeed, ySpeed);
    if (linearSpeed > MAX_LINEAR_VELOCITY) {
      double scale = MAX_LINEAR_VELOCITY / linearSpeed;
      xSpeed *= scale;
      ySpeed *= scale;
    }
    omega = MathUtil.clamp(omega, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return !valid || (xController.atGoal() && yController.atGoal() && headingController.atGoal());
  }
}
