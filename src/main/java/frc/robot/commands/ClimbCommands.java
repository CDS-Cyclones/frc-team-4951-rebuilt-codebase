package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;

public class ClimbCommands {

  public static Command climbUp(Climber climber) {
    return Commands.runEnd(() -> climber.run(0.8), () -> climber.stop());
  }

  public static Command climbDown(Climber climber) {
    return Commands.runEnd(() -> climber.run(-0.8), () -> climber.stop());
  }
}
