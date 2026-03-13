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
  private final SparkMax leftMotor = new SparkMax(kLeftShooterCANId, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(kRightShooterCANId, MotorType.kBrushless);
  private final SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
  private final SparkClosedLoopController rightController = rightMotor.getClosedLoopController();

  public ShooterIOSparkMax() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(kCurrentLimit);
    config.voltageCompensation(12);
    config.idleMode(IdleMode.kCoast);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kShooterKp.getAsDouble(), kShooterKi.getAsDouble(), kShooterKd.getAsDouble());

    // right config
    rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // left config (inverted)
    config.inverted(true);
    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftAppliedOutput = leftMotor.getAppliedOutput();
    inputs.rightAppliedOutput = rightMotor.getAppliedOutput();
    inputs.leftVelocityRPM = leftMotor.getEncoder().getVelocity();
    inputs.rightVelocityRPM = rightMotor.getEncoder().getVelocity();
  }

  @Override
  public void setPower(double power) {
    leftMotor.set(power);
    rightMotor.set(power);
  }

  @Override
  public void setVelocityRPM(double leftRPM, double rightRPM) {
    leftController.setSetpoint(leftRPM, ControlType.kVelocity);
    rightController.setSetpoint(rightRPM, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
