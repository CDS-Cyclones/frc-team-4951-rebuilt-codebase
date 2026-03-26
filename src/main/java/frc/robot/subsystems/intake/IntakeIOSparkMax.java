package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax motor;
  private final SparkMaxConfig config = new SparkMaxConfig();

  public IntakeIOSparkMax() {
    motor = new SparkMax(Constants.IntakeConstants.kCanId, MotorType.kBrushless);

    config.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12.0);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }
}
