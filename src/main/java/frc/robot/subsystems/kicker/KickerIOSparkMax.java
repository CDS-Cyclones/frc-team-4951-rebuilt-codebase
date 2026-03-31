package frc.robot.subsystems.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class KickerIOSparkMax implements KickerIO {
  private final SparkMax motor =
      new SparkMax(Constants.KickerConstants.kKickerCANId, MotorType.kBrushless);
  private final SparkClosedLoopController controller = motor.getClosedLoopController();

  public KickerIOSparkMax() {
    configureMotor(ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Constants.KickerConstants.kKickerKp.onChange(this::updateClosedLoopGains);
    Constants.KickerConstants.kKickerKi.onChange(this::updateClosedLoopGains);
    Constants.KickerConstants.kKickerKd.onChange(this::updateClosedLoopGains);
  }

  private void updateClosedLoopGains() {
    configureMotor(ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
  }

  private void configureMotor(ResetMode resetMode, PersistMode persistMode) {
    SparkMaxConfig configMain = new SparkMaxConfig();
    configMain.smartCurrentLimit(Constants.KickerConstants.kKickerCurrentLimit);
    configMain.voltageCompensation(12);
    configMain.idleMode(IdleMode.kCoast);
    configMain.inverted(false);
    configMain
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            Constants.KickerConstants.kKickerKp.getAsDouble(),
            Constants.KickerConstants.kKickerKi.getAsDouble(),
            Constants.KickerConstants.kKickerKd.getAsDouble());

    motor.configure(configMain, resetMode, persistMode);
  }

  private void applyVelocitySetpoint(double rpm) {
    double negRpm = -rpm;
    double ffVolts =
        Constants.KickerConstants.kKickerKs.getAsDouble() * Math.signum(rpm)
            + Constants.KickerConstants.kKickerKv.getAsDouble() * negRpm;
    controller.setSetpoint(
        negRpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setVelocityRPM(double RPM) {
    applyVelocitySetpoint(RPM);
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }
}
