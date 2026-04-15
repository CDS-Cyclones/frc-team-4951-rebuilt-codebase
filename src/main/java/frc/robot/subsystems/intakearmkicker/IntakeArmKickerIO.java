package frc.robot.subsystems.intakearmkicker;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmKickerIO {
  @AutoLog
  public static class IntakeArmKickerIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocityRPM = 0.0;
  }

  default void updateInputs(IntakeArmKickerIOInputs inputs) {}

  default void setPercent(double percent) {}

  default void stop() {
    setPercent(0.0);
  }
}
