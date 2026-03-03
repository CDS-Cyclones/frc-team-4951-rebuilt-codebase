package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class ManipulationCommands {

  public static Command toggleIntake(Intake intake) {
    return Commands.runEnd(
        () -> intake.run(Constants.IntakeConstants.intakeSpeed), intake::stop, intake);
  }
}
