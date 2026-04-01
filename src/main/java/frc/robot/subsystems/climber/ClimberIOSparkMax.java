package frc.robot.subsystems.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ClimberIOSparkMax implements ClimberIO {

  private final SparkMax motor =
      new SparkMax(Constants.ClimberConstants.kCanId, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder();
  private final RelativeEncoder relativeEncoder = motor.getEncoder();

  public ClimberIOSparkMax() {

    config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ClimberConstants.kCurrentStallLimit)
        .voltageCompensation(12.0);

    config
        .absoluteEncoder
        .positionConversionFactor(Constants.ClimberConstants.absolutePositionConversionFactor)
        .velocityConversionFactor(Constants.ClimberConstants.absoluteVelocityConversionFactor)
        .inverted(false);

    config
        .encoder
        .positionConversionFactor(Constants.ClimberConstants.relativePositionConversionFactor)
        .velocityConversionFactor(Constants.ClimberConstants.relativeVelocityConversionFactor);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.velocityRPM = relativeEncoder.getVelocity();
    inputs.absolutePositionDegrees = absoluteEncoder.getPosition();
    inputs.relativePositionDegrees = relativeEncoder.getPosition();
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }
}
