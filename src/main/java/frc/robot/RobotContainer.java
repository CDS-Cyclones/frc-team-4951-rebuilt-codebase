// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAimShootCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManipulationCommands;
import frc.robot.commands.OrbitCommand;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
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
  private final Shooter shooter;

  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

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
                new VisionIOLimelight(Constants.VisionConstants.camera0Name, drive::getRotation));
        intake = new Intake(new IntakeIOSparkMax());
        shooter = new Shooter(new ShooterIOSparkMax());
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
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
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
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    controller.leftBumper().whileTrue(new OrbitCommand(drive, () -> controller.getLeftX()));
    controller
        .rightBumper()
        .whileTrue(
            new AutoAimShootCommand(
                drive, shooter, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    controller.povUp().toggleOnTrue(ManipulationCommands.toggleIntake(intake));
    controller.povDown().toggleOnTrue(ManipulationCommands.shootFuel(shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    drive.setPose(driveSimulation.getSimulatedDriveTrainPose());
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }
}
