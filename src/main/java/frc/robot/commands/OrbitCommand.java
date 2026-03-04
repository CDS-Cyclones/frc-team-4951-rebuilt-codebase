// namesake from our israeli friends 1690

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class OrbitCommand extends Command {
  // tag ids for red and blue alliance
  private static final int[] RED_TAG_IDS = {9, 10};
  private static final int[] BLUE_TAG_IDS = {25, 26};

  // Driver input scales tangential motion around the orbit center.
  // drive input scales tangential motion through the center
  private static final double JOYSTICK_DEADBAND = 0.1;
  private static final double MAX_TANGENTIAL_SPEED_METERS_PER_SEC = 3.5;
  private static final double RIGHT_OFFSET_METERS = Units.inchesToMeters(3.0);
  // target radius of how far we wanna orbit from
  private static final double TARGET_RADIUS_METERS = 1.5;
  private static final double MIN_CONTROL_RADIUS_METERS = 0.2;
  private static final double RADIUS_KP = 2.0;
  private static final double MAX_RADIAL_SPEED_METERS_PER_SEC = 1.0;
  // heading control keeps the robot pointed at the tag midpoint while translating.
  private static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 8.0;
  private static final double HEADING_ERROR_SLOWDOWN_RADIANS = 0.6;
  private static final double MIN_TRANSLATION_SCALE = 0.45;

  private static final double HEADING_KP = 5.0;
  private static final double HEADING_KD = 0.4;
  private static final double HEADING_MAX_VELOCITY = 8.0;
  private static final double HEADING_MAX_ACCELERATION = 20.0;

  private final Drive drive;
  private final DoubleSupplier orbitInputSupplier;

  private final PIDController radiusController = new PIDController(RADIUS_KP, 0.0, 0.0);
  private final ProfiledPIDController headingController =
      new ProfiledPIDController(
          HEADING_KP,
          0.0,
          HEADING_KD,
          new TrapezoidProfile.Constraints(HEADING_MAX_VELOCITY, HEADING_MAX_ACCELERATION));

  private Translation2d orbitCenter;
  private boolean validOrbit = false;

  public OrbitCommand(Drive drive, DoubleSupplier orbitInputSupplier) {
    this.drive = drive;
    this.orbitInputSupplier = orbitInputSupplier;

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    validOrbit = false;

    // get the orbit center from the apriltag layout
    Optional<Translation2d> orbitCenterOptional = getAllianceOrbitCenter();
    if (orbitCenterOptional.isEmpty()) {
      DriverStation.reportWarning("OrbitCommand could not find the requested AprilTags.", false);
      return;
    }

    orbitCenter = orbitCenterOptional.get();

    Pose2d robotPose = drive.getPose();
    Translation2d robotOffset = robotPose.getTranslation().minus(orbitCenter);
    double startRadius = robotOffset.getNorm();

    if (startRadius < Math.pow(10, -5)) {
      DriverStation.reportWarning(
          "OrbitCommand cannot start while robot is at the orbit center.", false);
      return;
    }

    radiusController.reset();
    headingController.reset(robotPose.getRotation().getRadians());
    validOrbit = true;
  }

  @Override
  public void execute() {
    if (!validOrbit) {
      drive.stop();
      return;
    }

    Pose2d robotPose = drive.getPose();
    Translation2d robotOffset = robotPose.getTranslation().minus(orbitCenter);
    double currentRadius = robotOffset.getNorm();

    if (currentRadius < Math.pow(10, -5)) {
      drive.stop();
      return;
    }

    Translation2d radialUnit =
        new Translation2d(robotOffset.getX() / currentRadius, robotOffset.getY() / currentRadius);
    // mr edwards type physics (tangent direction would be the travel around the circle)
    Translation2d tangentUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX());

    // i stole this correction stuff
    double radialSpeed =
        MathUtil.clamp(
            radiusController.calculate(currentRadius, TARGET_RADIUS_METERS),
            -MAX_RADIAL_SPEED_METERS_PER_SEC,
            MAX_RADIAL_SPEED_METERS_PER_SEC);
    double requestedTangentialSpeed =
        MathUtil.applyDeadband(orbitInputSupplier.getAsDouble(), JOYSTICK_DEADBAND)
            * MAX_TANGENTIAL_SPEED_METERS_PER_SEC;

    Rotation2d desiredHeading = orbitCenter.minus(robotPose.getTranslation()).getAngle();
    // slow down when error gets too large so that we can stay infront of tags
    double headingErrorRadians = desiredHeading.minus(robotPose.getRotation()).getRadians();
    double translationScale =
        MathUtil.clamp(
            1.0 - (Math.abs(headingErrorRadians) / HEADING_ERROR_SLOWDOWN_RADIANS),
            MIN_TRANSLATION_SCALE,
            1.0);
    // vtotal^2 = vradial^2 + vt^2
    // this solves for max tangential component
    double maxTangentialSpeedFromLinearLimit =
        Math.sqrt(
            Math.max(
                0.0,
                Math.pow(drive.getMaxLinearSpeedMetersPerSec(), 2) - Math.pow(radialSpeed, 2)));
    // clamps so velocity never goes beyond linear speed
    double tangentialSpeed =
        MathUtil.clamp(
            requestedTangentialSpeed,
            -maxTangentialSpeedFromLinearLimit,
            maxTangentialSpeedFromLinearLimit);
    // constructs final field-relative velocity vector, vt component moves arobot around
    // radial moves toward or away from the center to keep the radius
    Translation2d fieldVelocity =
        tangentUnit.times(tangentialSpeed * translationScale).plus(radialUnit.times(radialSpeed));
    // heading correction to align robot w heading
    double headingFeedback =
        headingController.calculate(
            robotPose.getRotation().getRadians(), desiredHeading.getRadians());
    double headingFeedforward =
        tangentialSpeed / Math.max(currentRadius, MIN_CONTROL_RADIUS_METERS);
    double omega =
        MathUtil.clamp(
            headingFeedback + headingFeedforward,
            -MAX_ANGULAR_SPEED_RAD_PER_SEC,
            MAX_ANGULAR_SPEED_RAD_PER_SEC);
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldVelocity.getX(), fieldVelocity.getY(), omega, drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return !validOrbit;
  }

  private Optional<Translation2d> getAllianceOrbitCenter() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    int[] tagIds = alliance == Alliance.Red ? RED_TAG_IDS : BLUE_TAG_IDS;

    Optional<Pose3d> firstTag = Constants.VisionConstants.aprilTagLayout.getTagPose(tagIds[0]);
    Optional<Pose3d> secondTag = Constants.VisionConstants.aprilTagLayout.getTagPose(tagIds[1]);

    if (firstTag.isEmpty() || secondTag.isEmpty()) {
      return Optional.empty();
    }

    Translation2d firstTranslation = firstTag.get().getTranslation().toTranslation2d();
    Translation2d secondTranslation = secondTag.get().getTranslation().toTranslation2d();
    Translation2d midpoint = firstTranslation.interpolate(secondTranslation, 0.5);
    Rotation2d averageFacing =
        firstTag
            .get()
            .getRotation()
            .toRotation2d()
            .interpolate(secondTag.get().getRotation().toRotation2d(), 0.5);
    Translation2d rightOffset =
        new Translation2d(RIGHT_OFFSET_METERS, averageFacing.minus(Rotation2d.fromDegrees(90.0)));
    return Optional.of(midpoint.plus(rightOffset));
  }
}
