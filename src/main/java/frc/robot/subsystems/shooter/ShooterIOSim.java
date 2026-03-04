// dummy io implementation for simulation purposes
package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {
  private double appliedPower = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftAppliedOutput = appliedPower;
    inputs.RightAppliedOutput = appliedPower;
    inputs.leftVelocityRPM = appliedPower * 5000.0; // fake number
    inputs.rightVelocityRPM = appliedPower * 5000.0; // fake number
  }

  @Override
  public void setPower(double power) {
    appliedPower = power;
  }

  @Override
  public void stop() {
    appliedPower = 0.0;
  }
}
