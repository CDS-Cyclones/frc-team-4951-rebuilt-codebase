package frc.robot.subsystems.intakearmkicker;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class IntakeArmKickerIOSparkFlex implements IntakeArmKickerIO {
  private final SparkFlex motor =
      new SparkFlex(Constants.IntakeArmKickerConstants.kCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkFlexConfig config = new SparkFlexConfig();

  public IntakeArmKickerIOSparkFlex() {
    config
        .inverted(Constants.IntakeArmKickerConstants.kMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.IntakeArmKickerConstants.kCurrentLimit)
        .voltageCompensation(12.0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeArmKickerIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.velocityRPM = encoder.getVelocity();
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }
}
