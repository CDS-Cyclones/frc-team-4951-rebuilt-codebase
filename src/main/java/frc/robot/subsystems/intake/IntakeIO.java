package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocityRPM = 0.0;
    public int fuelCount = 0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPercent(double percent) {}

  default void stop() {
    setPercent(0.0);
  }

  default int getFuelCount() {
    return 0;
  }

  default boolean consumeFuel() {
    return false;
  }

  default void resetSimulationState() {}
}
