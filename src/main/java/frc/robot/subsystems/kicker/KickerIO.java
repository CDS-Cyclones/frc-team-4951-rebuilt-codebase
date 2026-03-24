package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocityRPM = 0.0;
  }

  default void updateInputs(KickerIOInputs inputs) {}

  default void setPercent(double percent) {}

  default void stop() {
    setPercent(0.0);
  }
}
