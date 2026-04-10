package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class HopperIOSparkMax implements HopperIO {
  private final SparkMax motor =
      new SparkMax(Constants.HopperConstants.kCanID, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();

  public HopperIOSparkMax() {
    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.HopperConstants.kCurrentLimit)
        .voltageCompensation(12.0)
        .inverted(false);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }
}
