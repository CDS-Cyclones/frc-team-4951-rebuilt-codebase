package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double leftAppliedOutput = 0.0;
    public double rightAppliedOutput = 0.0;
    public double leftVelocityRPM = 0.0;
    public double rightVelocityRPM = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setPower(double power) {}

  default void stop() {}

  default void setVelocityRPM(double leftRPM, double rightRPM) {}
}
