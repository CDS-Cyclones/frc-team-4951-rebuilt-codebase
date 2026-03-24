package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSparkMax implements ShooterIO {
  private final SparkMax mainMotor = new SparkMax(kMainShooterCANId, MotorType.kBrushless);
  private final SparkMax secondaryMotor =
      new SparkMax(kSecondaryShooterCANId, MotorType.kBrushless);
  private final SparkClosedLoopController mainController = mainMotor.getClosedLoopController();
  private final SparkClosedLoopController secondaryController =
      secondaryMotor.getClosedLoopController();

  public ShooterIOSparkMax() {
    SparkMaxConfig configMain = new SparkMaxConfig();
    configMain.smartCurrentLimit(kCurrentLimit);
    configMain.voltageCompensation(12);
    configMain.idleMode(IdleMode.kBrake);
    configMain
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            kShooterMainKp.getAsDouble(),
            kShooterMainKi.getAsDouble(),
            kShooterMainKd.getAsDouble());

    // main config
    mainMotor.configure(configMain, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig configSecondary = new SparkMaxConfig();
    configSecondary.smartCurrentLimit(kCurrentLimit);
    configSecondary.voltageCompensation(12);
    configSecondary.idleMode(IdleMode.kBrake);
    configSecondary.inverted(true);
    configSecondary
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            kShooterSecondaryKp.getAsDouble(),
            kShooterSecondaryKi.getAsDouble(),
            kShooterSecondaryKd.getAsDouble());

    secondaryMotor.configure(
        configSecondary, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.mainAppliedOutput = mainMotor.getAppliedOutput();
    inputs.secondaryAppliedOutput = secondaryMotor.getAppliedOutput();
    inputs.mainVelocityRPM = mainMotor.getEncoder().getVelocity();
    inputs.secondaryVelocityRPM = secondaryMotor.getEncoder().getVelocity();
  }

  @Override
  public void setPower(double power) {
    mainMotor.set(power);
    secondaryMotor.set(power);
  }

  @Override
  public void setVelocityRPM(double mainRPM, double secondaryRPM) {
    mainController.setSetpoint(mainRPM, ControlType.kVelocity);
    secondaryController.setSetpoint(secondaryRPM, ControlType.kVelocity);
  }

  @Override
  public void setMainVelocityRPM(double mainRPM) {
    mainController.setSetpoint(mainRPM, ControlType.kVelocity);
  }

  @Override
  public void setSecondaryVelocityRPM(double secondaryRPM) {
    secondaryController.setSetpoint(secondaryRPM, ControlType.kVelocity);
  }

  @Override
  public void setSecondaryPower(double power) {
    secondaryMotor.set(power);
  }

  @Override
  public void stop() {
    mainMotor.stopMotor();
    secondaryMotor.stopMotor();
  }
}
