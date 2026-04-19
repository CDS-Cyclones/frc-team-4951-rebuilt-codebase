package frc.robot.subsystems.intakearm;

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
  private final RelativeEncoder encoder = motor.getEncoder();

  public IntakeArmIOSparkMax() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(Constants.IntakeArmConstants.kMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.IntakeArmConstants.kCurrentLimit)
        .openLoopRampRate(2);
    config
        .softLimit
        .reverseSoftLimit(-12000)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(1000)
        .forwardSoftLimitEnabled(true);
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
    inputs.positionDegrees = encoder.getPosition();
    inputs.velocityDegreesPerSec = encoder.getVelocity();
  }

  // ik this is relative not abs
  public void getAbsoluteEncoderPosition() {
    encoder.getPosition();
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
