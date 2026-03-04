package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class IntakeIOSparkFlex implements IntakeIO {

  private final SparkFlex motor;
  private final SparkFlexConfig config = new SparkFlexConfig();

  public IntakeIOSparkFlex() {
    motor = new SparkFlex(Constants.IntakeConstants.kCanId, MotorType.kBrushless);

    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);

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
