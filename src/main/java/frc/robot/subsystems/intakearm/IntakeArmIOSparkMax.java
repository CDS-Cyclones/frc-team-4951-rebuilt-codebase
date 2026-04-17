package frc.robot.subsystems.intakearm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class IntakeArmIOSparkMax implements IntakeArmIO {
  private final SparkMax motor =
      new SparkMax(Constants.IntakeArmConstants.kCanId, MotorType.kBrushless);
  private final AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder();
  private final RelativeEncoder encoder = motor.getEncoder();

  public IntakeArmIOSparkMax() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(Constants.IntakeArmConstants.kMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.IntakeArmConstants.kCurrentLimit)
        .voltageCompensation(12.0);
    config
        .absoluteEncoder
        .positionConversionFactor(Constants.IntakeArmConstants.kAbsolutePositionConversionFactor)
        .velocityConversionFactor(Constants.IntakeArmConstants.kAbsoluteVelocityConversionFactor)
        .inverted(false);
    config
        .encoder
        .positionConversionFactor(Constants.IntakeArmConstants.kPositionConversionFactor)
        .velocityConversionFactor(Constants.IntakeArmConstants.kVelocityConversionFactor);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.absolutePositionDegrees = absoluteEncoder.getPosition();
    inputs.positionDegrees = encoder.getPosition();
    inputs.velocityDegreesPerSec = encoder.getVelocity();
  }

  public void getAbsoluteEncoderPosition() {
    absoluteEncoder.getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }

  @Override
  public void resetPositionDegrees(double positionDegrees) {
    encoder.setPosition(positionDegrees);
  }
}
