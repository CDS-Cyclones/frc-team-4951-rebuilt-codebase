package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ClimberIOSparkMax implements ClimberIO {

  private final SparkMax motor;
  private final SparkMaxConfig config = new SparkMaxConfig();

  public ClimberIOSparkMax() {
    motor = new SparkMax(Constants.ClimberConstants.kCanId, MotorType.kBrushless);

    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ClimberConstants.kCurrentStallLimit)
        .voltageCompensation(12.0);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }
}
