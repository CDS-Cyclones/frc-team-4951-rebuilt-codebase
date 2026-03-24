package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double mainAppliedOutput = 0.0;
    public double secondaryAppliedOutput = 0.0;
    public double mainVelocityRPM = 0.0;
    public double secondaryVelocityRPM = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setPower(double power) {}

  default void stop() {}

  default void setVelocityRPM(double mainRPM, double secondaryRPM) {}

  default void setSecondaryVelocityRPM(double secondaryRPM) {}

  default void setMainVelocityRPM(double mainRPM) {}

  default void setSecondaryPower(double power) {}
}
