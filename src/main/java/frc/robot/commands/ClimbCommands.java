package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;

public class ClimbCommands {
  public static Command climbUp(Climber climber) {
    return Commands.runEnd(
        () -> climber.run(Constants.ClimberConstants.kClimbPercent.getAsDouble()),
        climber::stop,
        climber);
  }

  public static Command climbDown(Climber climber) {
    return Commands.runEnd(
        () -> climber.run(-Constants.ClimberConstants.kClimbPercent.getAsDouble()),
        climber::stop,
        climber);
  }

  public static Command clearRung(Climber climber) {
    return climbDown(climber)
        .until(
            () ->
                climber.getAbsolutePositionDegrees()
                    >= Constants.ClimberConstants.kClearRung.getAsDouble())
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
