package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocityRPM = 0.0;
  }

  default void updateInputs(HopperIOInputs inputs) {}

  default void setPercent(double percent) {}

  default void stop() {
    setPercent(0.0);
  }
}
