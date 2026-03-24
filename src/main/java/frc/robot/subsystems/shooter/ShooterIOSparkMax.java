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
  private final SparkMax followerMotor = new SparkMax(kFollowerShooterCANId, MotorType.kBrushless);
  private final SparkClosedLoopController mainController = mainMotor.getClosedLoopController();
  private final SparkClosedLoopController followerController =
      followerMotor.getClosedLoopController();

  public ShooterIOSparkMax() {
    SparkMaxConfig configMain = new SparkMaxConfig();
    configMain.smartCurrentLimit(kCurrentLimit);
    configMain.voltageCompensation(12);
    configMain.idleMode(IdleMode.kCoast);
    configMain
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            kShooterMainKp.getAsDouble(),
            kShooterMainKi.getAsDouble(),
            kShooterMainKd.getAsDouble());

    // main config
    mainMotor.configure(configMain, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig configFollower = new SparkMaxConfig();
    configFollower.smartCurrentLimit(kCurrentLimit);
    configFollower.voltageCompensation(12);
    configFollower.idleMode(IdleMode.kCoast);
    configFollower.inverted(true);
    configFollower
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            kShooterFollowerKp.getAsDouble(),
            kShooterFollowerKi.getAsDouble(),
            kShooterFollowerKd.getAsDouble());

    followerMotor.configure(
        configFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.mainAppliedOutput = mainMotor.getAppliedOutput();
    inputs.followerAppliedOutput = followerMotor.getAppliedOutput();
    inputs.mainVelocityRPM = mainMotor.getEncoder().getVelocity();
    inputs.followerVelocityRPM = followerMotor.getEncoder().getVelocity();
  }

  @Override
  public void setPower(double power) {
    mainMotor.set(power);
    followerMotor.set(power);
  }

  @Override
  public void setFollowerPower(double power) {
    followerMotor.set(power);
  }

  @Override
  public void setVelocityRPM(double mainRPM, double followerRPM) {
    mainController.setSetpoint(mainRPM, ControlType.kVelocity);
    followerController.setSetpoint(followerRPM, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    mainMotor.stopMotor();
    followerMotor.stopMotor();
  }

  @Override
  public void stopFollower() {
    followerMotor.stopMotor();
  }
}
