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
import frc.robot.Robot;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakearm.IntakeArm;
import frc.robot.subsystems.intakearmkicker.IntakeArmKicker;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class ManipulationCommands {

  private static void runIntakeWithKicker(
      Intake intake, Kicker kicker, double intakePower, double kickerPower) {
    intake.run(intakePower);
    kicker.run(kickerPower);
  }

  public static Command toggleIntake(Intake intake, Kicker kicker) {
    return holdIntake(intake, kicker);
  }

  public static Command toggleIntake(
      Intake intake, Kicker kicker, IntakeArmKicker intakeArmKicker) {
    return Commands.runEnd(
        () -> {
          intake.run(Constants.IntakeConstants.intakeSpeed);
          kicker.run(Constants.IntakeConstants.kickerIntakeSpeed);
          intakeArmKicker.run(Constants.IntakeArmKickerConstants.kRunPercent);
        },
        () -> {
          intake.stop();
          kicker.stop();
          intakeArmKicker.stop();
        },
        intake,
        kicker,
        intakeArmKicker);
  }

  public static Command holdIntake(Intake intake, Kicker kicker) {
    return Commands.runEnd(
        () ->
            runIntakeWithKicker(
                intake,
                kicker,
                Constants.IntakeConstants.intakeSpeed,
                Constants.IntakeConstants.kickerIntakeSpeed),
        () -> {
          intake.stop();
          kicker.stop();
        },
        intake,
        kicker);
  }

  public static Command startIntake(Intake intake, Kicker kicker) {
    return Commands.runOnce(
        () ->
            runIntakeWithKicker(
                intake,
                kicker,
                Constants.IntakeConstants.intakeSpeed,
                Constants.IntakeConstants.kickerIntakeSpeed),
        intake,
        kicker);
  }

  public static Command outtake(Intake intake, Kicker kicker) {
    return Commands.runEnd(
        () ->
            runIntakeWithKicker(
                intake,
                kicker,
                Constants.IntakeConstants.outtakeSpeed,
                Constants.IntakeConstants.kickerOuttakeSpeed),
        () -> {
          intake.stop();
          kicker.stop();
        },
        intake,
        kicker);
  }

  public static Command outtake(Intake intake) {
    return Commands.runEnd(
        () -> intake.run(Constants.IntakeConstants.outtakeSpeed), () -> intake.stop(), intake);
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

  public static Command releaseIntakeArm(IntakeArm intakeArm, Climber climber) {
    return Commands.run(intakeArm::release, intakeArm)
        .until(
            () ->
                intakeArm.getAbsolutePositionDegrees()
                    >= Constants.IntakeArmConstants.kDeployedPositionDegrees)
        .onlyIf(
            () ->
                Math.abs(
                        climber.getAbsolutePositionDegrees()
                            - Constants.ClimberConstants.kClimbStowedPositionDegrees)
                    <= 15.0);
  }

  public static Command stowIntakeArm(IntakeArm intakeArm) {
    return Commands.run(intakeArm::stow, intakeArm)
        .until(
            () ->
                intakeArm.getAbsolutePositionDegrees()
                    <= Constants.IntakeArmConstants.kStowedPositionDegrees);
  }

  public static Command toggleIntakeArm(IntakeArm intakeArm, Climber climber) {
    double halfwayDegrees =
        (Constants.IntakeArmConstants.kStowedPositionDegrees
                + Constants.IntakeArmConstants.kDeployedPositionDegrees)
            / 2.0;

    return Commands.either(
        stowIntakeArm(intakeArm),
        releaseIntakeArm(intakeArm, climber),
        () -> intakeArm.getAbsolutePositionDegrees() >= halfwayDegrees);
  }

  public static Command runHopper(Hopper hopper) {
    return Commands.runEnd(() -> hopper.run(0.05), hopper::stop, hopper);
  }

  public static Command pulseHopper(Hopper hopper) {
    Timer pulseTimer = new Timer();
    boolean[] forward = {true};
    return Commands.runEnd(
        () -> {
          if (!pulseTimer.isRunning()) {
            forward[0] = true;
            pulseTimer.restart();
            hopper.run(0.75);
            return;
          }

          if (pulseTimer.hasElapsed(0.3)) {
            forward[0] = !forward[0];
            pulseTimer.restart();
            hopper.run(forward[0] ? 0.75 : -0.75);
          }
        },
        () -> {
          pulseTimer.stop();
          hopper.stop();
        },
        hopper);
  }

  public static Command shootFuel(Intake intake, Shooter shooter, Kicker kicker, Hopper hopper) {
    return createRealShootCommand(
            intake,
            shooter,
            kicker,
            hopper,
            () -> Constants.ShooterConstants.kShootRPM.getAsDouble(),
            () -> true)
        .onlyIf(() -> true);
  }

  public static Command shootFuelInAuto(
      Intake intake, Shooter shooter, Kicker kicker, Hopper hopper) {
    return createRealShootCommand(
            intake,
            shooter,
            kicker,
            hopper,
            () -> Constants.ShooterConstants.k4mShootRPM,
            () -> true)
        .onlyIf(() -> Robot.isHubActive());
  }

  public static Command passFuel(Intake intake, Shooter shooter, Kicker kicker, Hopper hopper) {
    return createRealShootCommand(
        intake,
        shooter,
        kicker,
        hopper,
        () -> Constants.ShooterConstants.kPassRPM.getAsDouble(),
        () -> true);
  }

  public static Command shootFuel(
      Drive drive, Intake intake, Shooter shooter, Kicker kicker, Hopper hopper) {
    return shootFuel(
            drive,
            intake,
            shooter,
            kicker,
            hopper,
            () -> Constants.ShooterConstants.kShootRPM.getAsDouble(),
            () -> true)
        .onlyIf(() -> true);
  }

  public static Command passFuel(
      Drive drive, Intake intake, Shooter shooter, Kicker kicker, Hopper hopper) {
    return shootFuel(
            drive,
            intake,
            shooter,
            kicker,
            hopper,
            () -> Constants.ShooterConstants.kPassRPM.getAsDouble(),
            () -> true)
        .onlyIf(() -> true);
  }

  public static Command shootFuel(
      Drive drive,
      Intake intake,
      Shooter shooter,
      Kicker kicker,
      Hopper hopper,
      DoubleSupplier rpmSupplier,
      BooleanSupplier canShootSupplier) {
    Command realShootCommand =
        createRealShootCommand(intake, shooter, kicker, hopper, rpmSupplier, canShootSupplier)
            .onlyIf(() -> true);

    if (Constants.currentMode != Mode.SIM) {
      return realShootCommand;
    }

    Timer launchTimer = new Timer();
    Command simShootCommand =
        Commands.runEnd(
            () -> {
              if (!launchTimer.isRunning()) {
                launchTimer.start();
              }

              double rpm = rpmSupplier.getAsDouble();
              if (rpm <= 0.0 || !shooter.isAtSpeed() || !intake.hasFuel()) {
                launchTimer.restart();
                return;
              }

              if (!launchTimer.advanceIfElapsed(
                  Constants.ShooterConstants.kSimLaunchPeriodSeconds)) {
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
                          Degrees.of(
                              Constants.ShooterConstants.kSimLaunchAngleDegrees.getAsDouble()))
                      .withProjectileTrajectoryDisplayCallBack(
                          trajectory ->
                              Logger.recordOutput(
                                  "FieldSimulation/FuelShotTrajectory",
                                  trajectory.toArray(Pose3d[]::new)),
                          trajectory ->
                              Logger.recordOutput(
                                  "FieldSimulation/FuelShotTrajectory",
                                  trajectory.toArray(Pose3d[]::new)));

              Logger.recordOutput("FieldSimulation/FuelShotTrajectory", emptyTrajectory);
              SimulatedArena.getInstance().addGamePieceProjectile(projectile);
            },
            () -> Logger.recordOutput("FieldSimulation/FuelShotTrajectory", new Pose3d[] {}));

    return Commands.parallel(realShootCommand, simShootCommand);
  }

  private static Command createRealShootCommand(
      Intake intake,
      Shooter shooter,
      Kicker kicker,
      Hopper hopper,
      DoubleSupplier rpmSupplier,
      BooleanSupplier canShootSupplier) {
    Timer hopperPulseTimer = new Timer();
    boolean[] hopperForward = {true};

    return Commands.runEnd(
        () -> {
          if (!canShootSupplier.getAsBoolean()) {
            intake.stop();
            shooter.stop();
            kicker.stop();
            hopper.stop();
            hopperPulseTimer.stop();
            hopperPulseTimer.reset();
            hopperForward[0] = true;
            return;
          }

          shooter.setVelocityRPM(rpmSupplier.getAsDouble());
          if (shooter.isMainAtSpeed()) {
            intake.run(Constants.IntakeConstants.kShootingSpeed);
            kicker.run(Constants.KickerConstants.kKickerPercentage);
            if (!hopperPulseTimer.isRunning()) {
              hopperForward[0] = true;
              hopperPulseTimer.restart();
              hopper.run(0.75);
            } else if (hopperPulseTimer.hasElapsed(0.3)) {
              hopperForward[0] = !hopperForward[0];
              hopperPulseTimer.restart();
              hopper.run(hopperForward[0] ? 0.75 : -0.75);
            }
          } else {
            intake.stop();
            kicker.stop();
            hopper.stop();
            hopperPulseTimer.stop();
            hopperPulseTimer.reset();
            hopperForward[0] = true;
          }
        },
        () -> {
          intake.stop();
          shooter.stop();
          kicker.stop();
          hopper.stop();
          hopperPulseTimer.stop();
          hopperPulseTimer.reset();
          hopperForward[0] = true;
        },
        intake,
        shooter,
        kicker,
        hopper);
  }
}
