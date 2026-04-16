package frc.robot.subsystems.intakearm;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {

  @AutoLog
  public static class IntakeArmIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double absolutePositionDegrees = 0.0;
    public double positionDegrees = 0.0;
    public double velocityDegreesPerSec = 0.0;
  }

  default void updateInputs(IntakeArmIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setPercent(double percent) {}

  default void stop() {
    setVoltage(0.0);
    setPercent(0.0);
  }

  default void resetPositionDegrees(double positionDegrees) {}
}
