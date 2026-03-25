package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class ManipulationCommands {
  private static void runIntakeWithKicker(Intake intake, Kicker kicker, double power) {
    intake.run(power);
    kicker.run(power);
  }

  public static Command toggleIntake(Intake intake, Kicker kicker) {
    return holdIntake(intake, kicker);
  }

  public static Command holdIntake(Intake intake, Kicker kicker) {
    return Commands.runEnd(
        () -> runIntakeWithKicker(intake, kicker, Constants.IntakeConstants.intakeSpeed),
        () -> {
          intake.stop();
          kicker.stop();
        },
        intake,
        kicker);
  }

  public static Command startIntake(Intake intake, Kicker kicker) {
    return Commands.runOnce(
        () -> runIntakeWithKicker(intake, kicker, Constants.IntakeConstants.intakeSpeed),
        intake,
        kicker);
  }

  public static Command stopIntake(Intake intake, Kicker kicker) {
    return Commands.runOnce(
        () -> {
          intake.stop();
          kicker.stop();
        },
        intake,
        kicker);
  }

  public static Command shootFuel(Shooter shooter, Kicker kicker) {
    return shootFuel(shooter, kicker, () -> Constants.ShooterConstants.kShootRPM.getAsDouble());
  }

  public static Command shootFuel(Shooter shooter, Kicker kicker, DoubleSupplier rpmSupplier) {
    return shootFuel(shooter, kicker, rpmSupplier, () -> true);
  }

  public static Command shootFuel(
      Shooter shooter,
      Kicker kicker,
      DoubleSupplier rpmSupplier,
      BooleanSupplier canShootSupplier) {
    return Commands.runEnd(
        () -> {
          if (!canShootSupplier.getAsBoolean()) {
            shooter.stop();
            kicker.stop();
            return;
          }

          double rpm = rpmSupplier.getAsDouble();
          if (shooter.isAtSpeed()) {
            shooter.setVelocityRPM(rpm);
            kicker.run(Constants.ShooterConstants.kKickerIndexPercent);
          } else {
            shooter.stop();
            kicker.stop();
          }
        },
        () -> {
          shooter.stop();
          kicker.stop();
        },
        shooter,
        kicker);
  }

  public static Command shootFuelSim(Drive drive, Shooter shooter) {
    return Commands.none();
  }

  public static Command shootFuelSim(Drive drive, Shooter shooter, Intake intake) {
    return shootFuelSim(
        drive, shooter, intake, () -> Constants.ShooterConstants.kShootRPM.getAsDouble());
  }

  public static Command shootFuelSim(
      Drive drive, Shooter shooter, Intake intake, DoubleSupplier rpmSupplier) {
    if (Constants.currentMode != Mode.SIM) {
      return Commands.none();
    }

    Timer launchTimer = new Timer();
    return Commands.runEnd(
        () -> {
          if (!launchTimer.isRunning()) {
            launchTimer.start();
          }

          double rpm = rpmSupplier.getAsDouble();
          if (rpm <= 0.0 || !shooter.isAtSpeed() || !intake.hasFuel()) {
            launchTimer.restart();
            return;
          }

          if (!launchTimer.advanceIfElapsed(Constants.ShooterConstants.kSimLaunchPeriodSeconds)) {
            return;
          }

          if (!intake.consumeFuel()) {
            launchTimer.restart();
            return;
          }

          Pose3d[] emptyTrajectory = new Pose3d[] {};
          Rotation2d heading = drive.getRotation();
          double simReferenceRpm = Constants.ShooterConstants.kSimReferenceRPM.getAsDouble();
          double launchSpeedScale = simReferenceRpm > 1e-6 ? rpm / simReferenceRpm : 0.0;
          double launchSpeedMetersPerSecond =
              Constants.ShooterConstants.kSimLaunchVelocityMetersPerSecond.getAsDouble()
                  * launchSpeedScale;

          var projectile =
              new RebuiltFuelOnFly(
                      drive.getPose().getTranslation(),
                      new Translation2d(
                          Constants.ShooterConstants.kSimLaunchForwardOffsetMeters, 0.0),
                      drive.getRobotRelativeSpeeds(),
                      heading,
                      Meters.of(Constants.ShooterConstants.kSimLaunchHeightMeters),
                      MetersPerSecond.of(launchSpeedMetersPerSecond),
                      Degrees.of(Constants.ShooterConstants.kSimLaunchAngleDegrees.getAsDouble()))
                  .withProjectileTrajectoryDisplayCallBack(
                      trajectory ->
                          Logger.recordOutput(
                              "FieldSimulation/FuelShotTrajectory",
                              trajectory.toArray(Pose3d[]::new)),
                      trajectory -> {
                        Logger.recordOutput(
                            "FieldSimulation/FuelShotTrajectory",
                            trajectory.toArray(Pose3d[]::new));
                      });

          Logger.recordOutput("FieldSimulation/FuelShotTrajectory", emptyTrajectory);
          SimulatedArena.getInstance().addGamePieceProjectile(projectile);
        },
        () -> Logger.recordOutput("FieldSimulation/FuelShotTrajectory", new Pose3d[] {}));
  }
}
