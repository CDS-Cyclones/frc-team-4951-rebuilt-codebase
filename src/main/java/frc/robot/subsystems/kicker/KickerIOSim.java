package frc.robot.subsystems.kicker;

public class KickerIOSim implements KickerIO {
  private double appliedPower = 0.0;
  private static final double kMaxRpm = 6000.0;

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.appliedVolts = appliedPower * 12.0;
    inputs.currentAmps = Math.abs(appliedPower) * 20.0;
    inputs.velocityRPM = appliedPower * kMaxRpm;
  }

  @Override
  public void setPercent(double percent) {
    appliedPower = percent;
  }
}
