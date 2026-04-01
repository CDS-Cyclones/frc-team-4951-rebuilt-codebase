package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;

public class ClimbCommands {
  private static final double kClimbPercent = 0.8;

  public static Command climbUp(Climber climber) {
    return Commands.runEnd(() -> climber.run(kClimbPercent), climber::stop, climber);
  }

  public static Command climbDown(Climber climber) {
    return Commands.runEnd(() -> climber.run(-kClimbPercent), climber::stop, climber);
  }

  public static Command clearRung(Climber climber) {
    return climbDown(climber)
        .until(() -> climber.getAbsolutePositionDegrees() >= Constants.ClimberConstants.kClearRung)
        .andThen(climber::stop);
  }

  public static Command climbUpFor(Climber climber, double seconds) {
    return climbUp(climber).withTimeout(seconds).andThen(() -> climber.run(0.0));
  }

  public static Command climbDownFor(Climber climber, double seconds) {
    return climbDown(climber).withTimeout(seconds).andThen(() -> climber.run(0.0));
  }

  public static Command stopClimb(Climber climber) {
    return Commands.runOnce(() -> climber.stop());
  }
}
