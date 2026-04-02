// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManipulationCommands;
import frc.robot.commands.TestCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.kicker.KickerIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Kicker kicker;
  private final Shooter shooter;
  private final Climber climber;

  private SwerveDriveSimulation driveSimulation = null;
  private boolean invertStuff = false;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController testController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(Constants.VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(Constants.VisionConstants.camera1Name, drive::getRotation));
        intake = new Intake(new IntakeIOSparkMax());
        kicker = new Kicker(new KickerIOSparkMax());
        shooter = new Shooter(new ShooterIOSparkMax());
        climber = new Climber(new ClimberIOSparkMax());
        break;

      case SIM:
        this.driveSimulation =
            new SwerveDriveSimulation(
                Constants.DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "camera",
                    Constants.VisionConstants.botToCamTransformSim,
                    driveSimulation::getSimulatedDriveTrainPose));
        intake = new Intake(new IntakeIOSim(driveSimulation));
        kicker = new Kicker(new KickerIOSim());
        shooter = new Shooter(new ShooterIOSim());
        climber = new Climber(new ClimberIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        kicker = new Kicker(new KickerIO() {});
        shooter = new Shooter(new ShooterIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand("intakeStart", ManipulationCommands.startIntake(intake, kicker));
    NamedCommands.registerCommand("intakeStop", ManipulationCommands.stopIntake(intake, kicker));
    NamedCommands.registerCommand(
        "climbUp", ClimbCommands.climbUpFor(climber, Constants.ClimberConstants.kSecondsToClimb));
    NamedCommands.registerCommand(
        "shootFuelAuto",
        ManipulationCommands.shootFuel(
            drive,
            intake,
            shooter,
            kicker,
            () -> Constants.ShooterConstants.kAutoShootRPM.getAsDouble(),
            () -> true));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "DRIVE BACKWARDS AND SHOOT",
        DriveCommands.joystickDrive(drive, () -> -0.5, () -> 0.0, () -> 0.0)
            .withTimeout(1.50)
            .andThen(Commands.runOnce(drive::stop, drive))
            .andThen(ManipulationCommands.shootFuelInAuto(intake, shooter, kicker)));

    autoChooser.addOption(
        "DRIVE BACKWARDS, SHOOT, CLIMB",
        Commands.parallel(
                DriveCommands.joystickDrive(drive, () -> 0.5, () -> 0.0, () -> 0.0)
                    .withTimeout(5.0),
                ClimbCommands.climbDownFor(climber, 4.25))
            .andThen(Commands.runOnce(drive::stop, drive))
            .andThen(ClimbCommands.stopClimb(climber))
            .andThen(ManipulationCommands.shootFuelInAuto(intake, shooter, kicker).withTimeout(3.5))
            .andThen(
                ClimbCommands.climbUpFor(climber, Constants.ClimberConstants.kSecondsToClimb)));
    autoChooser.addOption("test climb", ClimbCommands.climbDownFor(climber, 4.25));

    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    ////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////// MAIN CONTROLLER ///////////////////////////////////////
    /// ///////////////////////////////////////////////////////////////////////////////////////

    // Default command, normal field-relative drive
    final Command normalDriveCommand =
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX());
    final Command invertedDriveCommand =
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX());
    drive.setDefaultCommand(normalDriveCommand);
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : drive::zeroYaw;

    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // Orbit hub when left bumper is held
    // controller.leftBumper().whileTrue(new OrbitCommand(drive, () -> controller.getLeftX()));

    // Shoot in place when right bumper is held
    controller.rightBumper().whileTrue(ManipulationCommands.shootFuel(intake, shooter, kicker));
    controller.rightTrigger().whileTrue(ManipulationCommands.passFuel(intake, shooter, kicker));
    operatorController
        .rightBumper()
        .whileTrue(ManipulationCommands.shootFuel(intake, shooter, kicker));
    operatorController
        .rightTrigger()
        .whileTrue(ManipulationCommands.passFuel(intake, shooter, kicker));
    operatorController.povUp().whileTrue(ClimbCommands.climbUp(climber));
    operatorController.povDown().whileTrue(ClimbCommands.climbDown(climber));
    operatorController.a().toggleOnTrue(ManipulationCommands.toggleIntake(intake, kicker));
    operatorController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  invertStuff = !invertStuff;
                  drive.setDefaultCommand(invertStuff ? invertedDriveCommand : normalDriveCommand);
                },
                drive));

    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  invertStuff = !invertStuff;
                  drive.setDefaultCommand(invertStuff ? invertedDriveCommand : normalDriveCommand);
                },
                drive));
    // Toggle Intake with A button
    controller.a().toggleOnTrue(ManipulationCommands.toggleIntake(intake, kicker));

    controller
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d(Units.degreesToRadians(65))));

    controller.povUp().whileTrue((ClimbCommands.climbUp(climber)));
    controller.povDown().whileTrue((ClimbCommands.climbDown(climber)));
    controller.povLeft().whileTrue((ManipulationCommands.outtake(intake, kicker)));
    ////////////////////////////////////////////////////////////////////////////////////////////
    /// ///////////////////////////////// TEST CONTROLLER ///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    testController.a().whileTrue(TestCommands.holdIntake(intake, 0.55));
    testController
        .b()
        .whileTrue(
            TestCommands.holdKickerVelocity(
                kicker, () -> Constants.KickerConstants.kKickerShootRPM.getAsDouble()));
    testController.povUp().whileTrue(TestCommands.holdKicker(kicker, -0.55));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void resetPoseForAlliance() {
    drive.resetPoseForAlliance();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(7, 7, new Rotation2d()));
    drive.setPose(driveSimulation.getSimulatedDriveTrainPose());
    SimulatedArena.getInstance().resetFieldForAuto();
    intake.resetSimulationState();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }
}
