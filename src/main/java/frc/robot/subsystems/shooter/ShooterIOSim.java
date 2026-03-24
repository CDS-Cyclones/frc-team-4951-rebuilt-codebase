// dummy io implementation for simulation purposes
package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.RobotBase;

public class ShooterIOSim implements ShooterIO {
  private double mainAppliedPower = 0.0;
  private double followerAppliedPower = 0.0;
  private double mainVelocityRPM = 0.0;
  private double followerVelocityRPM = 0.0;
  private boolean wasRunning = false;
  private static final double kMaxRpm = 5000.0;
  private static final double kVelocityAlpha = 0.2;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.mainAppliedOutput = mainAppliedPower;
    inputs.followerAppliedOutput = followerAppliedPower;
    inputs.mainVelocityRPM = mainVelocityRPM;
    inputs.followerVelocityRPM = followerVelocityRPM;

    boolean isRunning = Math.abs(mainVelocityRPM) > 10.0 || Math.abs(followerVelocityRPM) > 10.0;

    if (RobotBase.isSimulation() && isRunning != wasRunning) {
      System.out.println("Shooter " + (isRunning ? "ON" : "OFF"));
      wasRunning = isRunning;
    }
  }

  @Override
  public void setPower(double power) {
    mainAppliedPower = power;
    followerAppliedPower = power;
    mainVelocityRPM = power * kMaxRpm;
    followerVelocityRPM = power * kMaxRpm;
  }

  @Override
  public void setFollowerPower(double power) {
    followerAppliedPower = power;
    followerVelocityRPM = power * kMaxRpm;
  }

  @Override
  public void setVelocityRPM(double mainRPM, double followerRPM) {
    mainAppliedPower = mainRPM / kMaxRpm;
    followerAppliedPower = followerRPM / kMaxRpm;
    mainVelocityRPM += (mainRPM - mainVelocityRPM) * kVelocityAlpha;
    followerVelocityRPM += (followerRPM - followerVelocityRPM) * kVelocityAlpha;
  }

  @Override
  public void stop() {
    mainAppliedPower = 0.0;
    followerAppliedPower = 0.0;
    mainVelocityRPM = 0.0;
    followerVelocityRPM = 0.0;
  }

  @Override
  public void stopFollower() {
    followerAppliedPower = 0.0;
    followerVelocityRPM = 0.0;
  }
}
