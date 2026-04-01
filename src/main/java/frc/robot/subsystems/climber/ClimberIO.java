package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocityRPM = 0.0;
    public int fuelCount = 0;
    public double absolutePositionDegrees = 0.0;
    public double relativePositionDegrees = 0.0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

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
