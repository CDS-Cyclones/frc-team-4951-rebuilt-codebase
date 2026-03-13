package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Drives with normal translation controls while auto-aiming robot yaw at the speaker and scheduling
 * shooter RPM from distance.
 */
public class AutoAimShootCommand extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final ProfiledPIDController headingController =
      new ProfiledPIDController(
          Constants.AutoAimShootConstants.aimKp,
          0.0,
          Constants.AutoAimShootConstants.aimKd,
          new TrapezoidProfile.Constraints(
              Constants.AutoAimShootConstants.aimMaxVelocityRadPerSec,
              Constants.AutoAimShootConstants.aimMaxAccelerationRadPerSecSq));

  public AutoAimShootCommand(
      Drive drive, Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    headingController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Optional<Translation2d> targetOptional = getAllianceTarget();
    if (targetOptional.isEmpty()) {
      drive.stop();
      shooter.stop();
      DriverStation.reportWarning("AutoAimShoot could not find target tags.", false);
      return;
    }

    Translation2d target = targetOptional.get();
    Pose2d robotPose = drive.getPose();

    Translation2d linearVelocity = getLinearVelocityFromJoysticks();
    Rotation2d desiredHeading = target.minus(robotPose.getTranslation()).getAngle();
    double omega =
        MathUtil.clamp(
            headingController.calculate(
                robotPose.getRotation().getRadians(), desiredHeading.getRadians()),
            -Constants.AutoAimShootConstants.maxAimOmegaRadPerSec,
            Constants.AutoAimShootConstants.maxAimOmegaRadPerSec);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

    double distanceMeters = robotPose.getTranslation().getDistance(target);
    double rpm = getShotRpm(distanceMeters);
    shooter.setVelocityRPM(rpm);

    Logger.recordOutput("AutoAimShoot/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("AutoAimShoot/DistanceMeters", distanceMeters);
    Logger.recordOutput("AutoAimShoot/SetpointRPM", rpm);
    Logger.recordOutput("AutoAimShoot/DesiredHeadingDeg", desiredHeading.getDegrees());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private Translation2d getLinearVelocityFromJoysticks() {
    double magnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
            Constants.AutoAimShootConstants.driveDeadband);
    Rotation2d direction =
        new Rotation2d(Math.atan2(ySupplier.getAsDouble(), xSupplier.getAsDouble()));
    magnitude = magnitude * magnitude;

    return new Pose2d(Translation2d.kZero, direction)
        .transformBy(new Transform2d(magnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  private Optional<Translation2d> getAllianceTarget() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    int[] targetTagIds =
        alliance == Alliance.Red
            ? Constants.AutoAimShootConstants.RED_TARGET_TAG_IDS
            : Constants.AutoAimShootConstants.BLUE_TARGET_TAG_IDS;

    Optional<Pose3d> firstTag =
        Constants.VisionConstants.aprilTagLayout.getTagPose(targetTagIds[0]);
    Optional<Pose3d> secondTag =
        Constants.VisionConstants.aprilTagLayout.getTagPose(targetTagIds[1]);
    if (firstTag.isEmpty() || secondTag.isEmpty()) {
      return Optional.empty();
    }

    Translation2d firstTranslation = firstTag.get().getTranslation().toTranslation2d();
    Translation2d secondTranslation = secondTag.get().getTranslation().toTranslation2d();
    return Optional.of(firstTranslation.interpolate(secondTranslation, 0.5));
  }

  private double getShotRpm(double distanceMeters) {
    double clampedDistance =
        MathUtil.clamp(
            distanceMeters,
            Constants.AutoAimShootConstants.minShotDistanceMeters,
            Constants.AutoAimShootConstants.maxShotDistanceMeters);
    return MathUtil.interpolate(
        Constants.AutoAimShootConstants.minShotRpm,
        Constants.AutoAimShootConstants.maxShotRpm,
        (clampedDistance - Constants.AutoAimShootConstants.minShotDistanceMeters)
            / (Constants.AutoAimShootConstants.maxShotDistanceMeters
                - Constants.AutoAimShootConstants.minShotDistanceMeters));
  }
}
