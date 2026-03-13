// dummy io implementation for simulation purposes
package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.RobotBase;

public class ShooterIOSim implements ShooterIO {
  private double leftAppliedPower = 0.0;
  private double rightAppliedPower = 0.0;
  private double leftVelocityRPM = 0.0;
  private double rightVelocityRPM = 0.0;
  private boolean wasRunning = false;
  private static final double kMaxRpm = 5000.0;
  private static final double kVelocityAlpha = 0.2;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftAppliedOutput = leftAppliedPower;
    inputs.rightAppliedOutput = rightAppliedPower;
    inputs.leftVelocityRPM = leftVelocityRPM;
    inputs.rightVelocityRPM = rightVelocityRPM;

    boolean isRunning = Math.abs(leftVelocityRPM) > 10.0 || Math.abs(rightVelocityRPM) > 10.0;

    if (RobotBase.isSimulation() && isRunning != wasRunning) {
      System.out.println("Shooter " + (isRunning ? "ON" : "OFF"));
      wasRunning = isRunning;
    }
  }

  @Override
  public void setPower(double power) {
    leftAppliedPower = power;
    rightAppliedPower = power;
    leftVelocityRPM = power * kMaxRpm;
    rightVelocityRPM = power * kMaxRpm;
  }

  @Override
  public void setVelocityRPM(double leftRPM, double rightRPM) {
    leftAppliedPower = leftRPM / kMaxRpm;
    rightAppliedPower = rightRPM / kMaxRpm;
    leftVelocityRPM += (leftRPM - leftVelocityRPM) * kVelocityAlpha;
    rightVelocityRPM += (rightRPM - rightVelocityRPM) * kVelocityAlpha;
  }

  @Override
  public void stop() {
    leftAppliedPower = 0.0;
    rightAppliedPower = 0.0;
    leftVelocityRPM = 0.0;
    rightVelocityRPM = 0.0;
  }
}
