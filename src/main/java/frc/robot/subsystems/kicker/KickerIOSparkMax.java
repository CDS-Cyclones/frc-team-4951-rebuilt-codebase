package frc.robot.subsystems.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class KickerIOSparkMax implements KickerIO {
  private final SparkMax motor;
  private final SparkMaxConfig config = new SparkMaxConfig();

  public KickerIOSparkMax() {
    motor = new SparkMax(Constants.ShooterConstants.kKickerCANId, MotorType.kBrushless);
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ShooterConstants.kKickerCurrentLimit)
        .voltageCompensation(12.0)
        .inverted(true);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }
}
