package frc.robot.subsystems.intakearm;

import static frc.robot.Constants.IntakeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
  private final IntakeArmIO io;
  private final IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          kKp.getAsDouble(),
          kKi.getAsDouble(),
          kKd.getAsDouble(),
          new TrapezoidProfile.Constraints(
              kMaxVelocityDegreesPerSecond, kMaxAccelerationDegreesPerSecondSq));

  private double goalDegrees = kStowedPositionDegrees;

  public IntakeArm(IntakeArmIO io) {
    this.io = io;
    controller.setTolerance(kToleranceDegrees);
    controller.reset(kStowedPositionDegrees);
    io.resetPositionDegrees(kStowedPositionDegrees);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeArm", inputs);

    controller.setGoal(goalDegrees);
    double outputVolts = MathUtil.clamp(controller.calculate(inputs.positionDegrees), -12.0, 12.0);
    io.setVoltage(outputVolts);

    Logger.recordOutput("IntakeArm/GoalDegrees", goalDegrees);
    Logger.recordOutput("IntakeArm/SetpointDegrees", controller.getSetpoint().position);
    Logger.recordOutput(
        "IntakeArm/SetpointVelocityDegreesPerSec", controller.getSetpoint().velocity);
    Logger.recordOutput("IntakeArm/AtGoal", controller.atGoal());
  }

  public void deploy() {
    goalDegrees = kDeployedPositionDegrees;
  }

  public void release() {
    deploy();
  }

  public void stow() {
    goalDegrees = kStowedPositionDegrees;
  }

  public void setGoalDegrees(double degrees) {
    goalDegrees = degrees;
  }

  public double getGoalDegrees() {
    return goalDegrees;
  }

  public boolean isDeployed() {
    return Math.abs(goalDegrees - kDeployedPositionDegrees) < 1e-3;
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  public double getAbsolutePositionDegrees() {
    return inputs.absolutePositionDegrees;
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public void stop() {
    goalDegrees = inputs.positionDegrees;
    controller.reset(inputs.positionDegrees);
    io.stop();
  }
}
