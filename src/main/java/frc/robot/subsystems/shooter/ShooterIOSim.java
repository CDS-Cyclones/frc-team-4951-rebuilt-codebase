// dummy io implementation for simulation purposes
package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.RobotBase;

public class ShooterIOSim implements ShooterIO {
  private double mainAppliedPower = 0.0;
  private double secondaryAppliedPower = 0.0;
  private double mainVelocityRPM = 0.0;
  private double secondaryVelocityRPM = 0.0;
  private boolean wasRunning = false;
  private static final double kMaxRpm = 5000.0;
  private static final double kVelocityAlpha = 0.2;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.mainAppliedOutput = mainAppliedPower;
    inputs.secondaryAppliedOutput = secondaryAppliedPower;
    inputs.mainVelocityRPM = mainVelocityRPM;
    inputs.secondaryVelocityRPM = secondaryVelocityRPM;

    boolean isRunning = Math.abs(mainVelocityRPM) > 10.0 || Math.abs(secondaryVelocityRPM) > 10.0;

    if (RobotBase.isSimulation() && isRunning != wasRunning) {
      System.out.println("Shooter " + (isRunning ? "ON" : "OFF"));
      wasRunning = isRunning;
    }
  }

  @Override
  public void setPower(double power) {
    mainAppliedPower = power;
    secondaryAppliedPower = power;
    mainVelocityRPM = power * kMaxRpm;
    secondaryVelocityRPM = power * kMaxRpm;
  }

  @Override
  public void setSecondaryPower(double power) {
    secondaryAppliedPower = power;
    secondaryVelocityRPM = power * kMaxRpm;
  }

  @Override
  public void setVelocityRPM(double mainRPM, double secondaryRPM) {
    secondaryRPM = mainRPM;
    mainAppliedPower = mainRPM / kMaxRpm;
    secondaryAppliedPower = secondaryRPM / kMaxRpm;
    mainVelocityRPM += (mainRPM - mainVelocityRPM) * kVelocityAlpha;
    secondaryVelocityRPM += (secondaryRPM - secondaryVelocityRPM) * kVelocityAlpha;
  }

  @Override
  public void stop() {
    mainAppliedPower = 0.0;
    secondaryAppliedPower = 0.0;
    mainVelocityRPM = 0.0;
    secondaryVelocityRPM = 0.0;
  }
}
