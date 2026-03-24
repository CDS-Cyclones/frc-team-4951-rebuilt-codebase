package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double mainAppliedOutput = 0.0;
    public double followerAppliedOutput = 0.0;
    public double mainVelocityRPM = 0.0;
    public double followerVelocityRPM = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setPower(double power) {}

  default void stop() {}

  default void setVelocityRPM(double mainRPM, double followerRPM) {}

  default void setFollowerVelocityRPM(double followerRPM) {}

  default void setMainVelocityRPM(double mainRPM){}

  default void setFollowerMotorPower(double followerMotorPower){}

  default void stopFollower() {}

  default void setFollowerPower(double power) {}

}
