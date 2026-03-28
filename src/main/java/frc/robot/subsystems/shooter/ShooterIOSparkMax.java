package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

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

public class ShooterIOSparkMax implements ShooterIO {
  private final SparkMax mainMotor = new SparkMax(kMainShooterCANId, MotorType.kBrushless);
  private final SparkMax secondaryMotor =
      new SparkMax(kSecondaryShooterCANId, MotorType.kBrushless);
  private final SparkClosedLoopController mainController = mainMotor.getClosedLoopController();

  public ShooterIOSparkMax() {
    configureMainMotor(ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kShooterMainKp.onChange(this::updateClosedLoopGains);
    kShooterMainKi.onChange(this::updateClosedLoopGains);
    kShooterMainKd.onChange(this::updateClosedLoopGains);

    SparkMaxConfig configSecondary = new SparkMaxConfig();
    configSecondary.smartCurrentLimit(kCurrentLimit);
    configSecondary.voltageCompensation(12);
    configSecondary.idleMode(IdleMode.kCoast);
    configSecondary.follow(mainMotor, true);

    secondaryMotor.configure(
        configSecondary, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.mainAppliedOutput = mainMotor.getAppliedOutput();
    inputs.secondaryAppliedOutput = secondaryMotor.getAppliedOutput();
    inputs.mainVelocityRPM = -mainMotor.getEncoder().getVelocity();
    inputs.secondaryVelocityRPM = -secondaryMotor.getEncoder().getVelocity();
  }

  @Override
  public void setPower(double power) {
    mainMotor.set(power);
    secondaryMotor.set(power);
  }

  @Override
  public void setVelocityRPM(double mainRPM, double secondaryRPM) {
    applyVelocitySetpoint(mainRPM);
  }

  @Override
  public void setMainVelocityRPM(double mainRPM) {
    applyVelocitySetpoint(mainRPM);
  }

  @Override
  public void setSecondaryVelocityRPM(double secondaryRPM) {
    applyVelocitySetpoint(secondaryRPM);
  }

  @Override
  public void setSecondaryPower(double power) {
    mainMotor.set(power);
  }

  @Override
  public void stop() {
    mainMotor.stopMotor();
  }

  private void configureMainMotor(ResetMode resetMode, PersistMode persistMode) {
    SparkMaxConfig configMain = new SparkMaxConfig();
    configMain.smartCurrentLimit(kCurrentLimit);
    configMain.voltageCompensation(12);
    configMain.idleMode(IdleMode.kCoast);
    configMain.inverted(true);
    configMain
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            kShooterMainKp.getAsDouble(),
            kShooterMainKi.getAsDouble(),
            kShooterMainKd.getAsDouble());

    mainMotor.configure(configMain, resetMode, persistMode);
  }

  private void updateClosedLoopGains() {
    configureMainMotor(ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void applyVelocitySetpoint(double rpm) {
    double ffVolts =
        kShooterMainKs.getAsDouble() * Math.signum(rpm) + kShooterMainKv.getAsDouble() * rpm;
    mainController.setSetpoint(
        rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
  }
}
