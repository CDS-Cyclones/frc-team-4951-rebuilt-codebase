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
import java.util.Optional;

/**
 * Drives the robot to a specified distance from an AprilTag, facing the tag. The robot approaches
 * along the tag's normal (the direction the tag faces) so it arrives head-on.
 */
public class DriveToTagCommand extends Command {
  private static final double TRANSLATION_KP = 2.0;
  private static final double TRANSLATION_KD = 0.1;
  private static final double MAX_LINEAR_VELOCITY = 3.0; // m/s
  private static final double MAX_LINEAR_ACCELERATION = 3.0; // m/s^2

  private static final double HEADING_KP = 5.0;
  private static final double HEADING_KD = 0.4;
  private static final double MAX_ANGULAR_VELOCITY = 8.0; // rad/s
  private static final double MAX_ANGULAR_ACCELERATION = 20.0; // rad/s^2

  private static final double POSITION_TOLERANCE = 0.05; // meters
  private static final double HEADING_TOLERANCE = Math.toRadians(2.0); // radians

  private final Drive drive;
  private final int tagId;
  private final double distanceMeters;

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
   * Creates a new DriveToTagCommand.
   *
   * @param drive The drive subsystem.
   * @param tagId The AprilTag ID to drive toward.
   * @param distanceMeters The desired distance from the tag face in meters.
   */
  public DriveToTagCommand(Drive drive, int tagId, double distanceMeters) {
    this.drive = drive;
    this.tagId = tagId;
    this.distanceMeters = distanceMeters;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    valid = false;

    Optional<Pose3d> tagPoseOptional = Constants.VisionConstants.aprilTagLayout.getTagPose(tagId);
    if (tagPoseOptional.isEmpty()) {
      DriverStation.reportWarning(
          "DriveToTagCommand: AprilTag " + tagId + " not found in layout.", false);
      return;
    }

    Pose3d tagPose3d = tagPoseOptional.get();
    Translation2d tagPosition = tagPose3d.getTranslation().toTranslation2d();
    // The tag's rotation tells us which way the tag faces.
    // We want the robot to stand in front of the tag at the requested distance,
    // along the tag's forward (normal) direction, facing back toward the tag.
    Rotation2d tagFacing = tagPose3d.getRotation().toRotation2d();
    Translation2d offset = new Translation2d(distanceMeters, tagFacing);
    Translation2d goalPosition = tagPosition.plus(offset);
    // Robot should face the tag (opposite of the tag's forward direction)
    Rotation2d goalHeading = tagFacing.plus(new Rotation2d(Math.PI));

    targetPose = new Pose2d(goalPosition, goalHeading);

    Pose2d currentPose = drive.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    headingController.reset(currentPose.getRotation().getRadians());

    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    headingController.setTolerance(HEADING_TOLERANCE);

    valid = true;
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

    // Clamp linear speed to max
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
