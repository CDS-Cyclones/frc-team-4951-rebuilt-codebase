package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ManipulationCommands {

  public static Command toggleIntake(Intake intake) {
    return Commands.runEnd(
        () -> intake.run(Constants.IntakeConstants.intakeSpeed), intake::stop, intake);
  }

  public static Command shootFuel(Shooter shooter) {
    return Commands.runEnd(
        () -> shooter.setVelocityRPM(Constants.ShooterConstants.kShootRPM), shooter::stop, shooter);
  }
}
