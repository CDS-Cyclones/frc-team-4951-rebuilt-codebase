package frc.robot.subsystems.intakearmkicker;

public class IntakeArmKickerIOSim implements IntakeArmKickerIO {
  private static final double kNominalVoltage = 12.0;
  private static final double kMaxRpm = 6000.0;

  private double appliedPower = 0.0;

  @Override
  public void updateInputs(IntakeArmKickerIOInputs inputs) {
    inputs.appliedVolts = appliedPower * kNominalVoltage;
    inputs.currentAmps = Math.abs(appliedPower) * 15.0;
    inputs.velocityRPM = appliedPower * kMaxRpm;
  }

  @Override
  public void setPercent(double percent) {
    appliedPower = percent;
  }
}
