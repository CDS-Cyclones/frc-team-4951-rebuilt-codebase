package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Drives with normal translation controls while auto-aiming robot yaw at the hub and scheduling
 * shooter RPM from distance.
 */
public class AutoAimShootCommand extends ParallelCommandGroup {

  public AutoAimShootCommand(
      Drive drive, Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    AtomicReference<Double> shotRpm = new AtomicReference<>(0.0);
    AtomicReference<Rotation2d> desiredHeading = new AtomicReference<>(drive.getRotation());

    addCommands(
        Commands.run(
                () -> {
                  Optional<Translation2d> targetOptional = getAllianceTarget();
                  if (targetOptional.isEmpty()) {
                    shotRpm.set(0.0);
                    desiredHeading.set(drive.getRotation());
                    DriverStation.reportWarning("AutoAimShoot could not find target tags.", false);
                    return;
                  }

                  Translation2d target = targetOptional.get();
                  Pose2d robotPose = drive.getPose();
                  Rotation2d targetHeading = target.minus(robotPose.getTranslation()).getAngle();
                  desiredHeading.set(targetHeading);

                  double distanceMeters = robotPose.getTranslation().getDistance(target);
                  double rpm = getShotRpm(distanceMeters);
                  shotRpm.set(rpm);

                  Logger.recordOutput(
                      "AutoAimShoot/TargetPose", new Pose2d(target, Rotation2d.kZero));
                  Logger.recordOutput("AutoAimShoot/DistanceMeters", distanceMeters);
                  Logger.recordOutput("AutoAimShoot/SetpointRPM", rpm);
                  Logger.recordOutput("AutoAimShoot/DesiredHeadingDeg", targetHeading.getDegrees());
                })
            .finallyDo(
                () -> {
                  shotRpm.set(0.0);
                  desiredHeading.set(drive.getRotation());
                }),
        DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, desiredHeading::get),
        ManipulationCommands.shootFuel(shooter, shotRpm::get));
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
