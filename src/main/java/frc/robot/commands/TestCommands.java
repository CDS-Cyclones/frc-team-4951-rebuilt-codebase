package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class TestCommands {
  private static final double kDefaultTestPercent = 0.5;

  private TestCommands() {}

  public static Command holdShooter(Shooter shooter) {
    return holdShooter(shooter, kDefaultTestPercent);
  }

  public static Command holdShooter(Shooter shooter, double percent) {
    return Commands.runEnd(() -> shooter.setPower(percent), shooter::stop, shooter);
  }

  public static Command holdShooterVelocity(Shooter shooter, double rpm) {
    return holdShooterVelocity(shooter, () -> rpm);
  }

  public static Command holdShooterVelocity(Shooter shooter, DoubleSupplier rpmSupplier) {
    return Commands.runEnd(
        () -> shooter.setVelocityRPM(rpmSupplier.getAsDouble()), shooter::stop, shooter);
  }

  public static Command holdKicker(Kicker kicker) {
    return holdKicker(kicker, kDefaultTestPercent);
  }

  public static Command holdKicker(Kicker kicker, double percent) {
    return Commands.runEnd(() -> kicker.run(percent), kicker::stop, kicker);
  }

  public static Command holdIntake(Intake intake) {
    return holdIntake(intake, kDefaultTestPercent);
  }

  public static Command holdIntake(Intake intake, double percent) {
    return Commands.runEnd(() -> intake.run(percent), intake::stop, intake);
  }
}
