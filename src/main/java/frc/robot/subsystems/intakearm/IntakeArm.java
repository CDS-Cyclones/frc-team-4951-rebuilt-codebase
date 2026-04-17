package frc.robot.subsystems.intakearm;

import static frc.robot.Constants.IntakeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
  private final IntakeArmIO io;
  private final IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

  private double goalDegrees = kStowedPositionDegrees;
  private boolean goalSeekingEnabled = false;
  private boolean openLoopEnabled = false;
  private double openLoopPercent = 0.0;

  public IntakeArm(IntakeArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeArm", inputs);

    double commandedPercent = 0.0;
    if (openLoopEnabled) {
      commandedPercent = openLoopPercent;
    } else if (goalSeekingEnabled) {
      commandedPercent = getGoalSeekingPercent();
    }
    io.setPercent(commandedPercent);

    Logger.recordOutput("IntakeArm/GoalDegrees", goalDegrees);
    Logger.recordOutput("IntakeArm/AtGoal", atGoal());
    Logger.recordOutput("IntakeArm/GoalSeekingEnabled", goalSeekingEnabled);
    Logger.recordOutput("IntakeArm/ManualOpenLoopEnabled", openLoopEnabled);
    Logger.recordOutput("IntakeArm/OpenLoopPercent", openLoopPercent);
    Logger.recordOutput("IntakeArm/CommandedPercent", commandedPercent);
  }

  private double getGoalSeekingPercent() {
    double errorDegrees = goalDegrees - inputs.absolutePositionDegrees;
    if (Math.abs(errorDegrees) <= kToleranceDegrees) {
      return 0.0;
    }

    double percent =
        errorDegrees > 0.0
            ? kDeployOpenLoopPercent.getAsDouble()
            : kStowOpenLoopPercent.getAsDouble();
    return MathUtil.clamp(percent, -1.0, 1.0);
  }

  public void deploy() {
    goalSeekingEnabled = true;
    openLoopEnabled = false;
    goalDegrees = kDeployedPositionDegrees;
  }

  public void release() {
    deploy();
  }

  public void stow() {
    goalSeekingEnabled = true;
    openLoopEnabled = false;
    goalDegrees = kStowedPositionDegrees;
  }

  public void setGoalDegrees(double degrees) {
    goalSeekingEnabled = true;
    openLoopEnabled = false;
    goalDegrees = degrees;
  }

  public void runOpenLoop(double percent) {
    goalSeekingEnabled = false;
    openLoopEnabled = true;
    openLoopPercent = MathUtil.clamp(percent, -1.0, 1.0);
  }

  public double getGoalDegrees() {
    return goalDegrees;
  }

  public boolean isDeployed() {
    return Math.abs(inputs.absolutePositionDegrees - kDeployedPositionDegrees) <= kToleranceDegrees;
  }

  public boolean atGoal() {
    return Math.abs(goalDegrees - inputs.absolutePositionDegrees) <= kToleranceDegrees;
  }

  public double getAbsolutePositionDegrees() {
    return inputs.absolutePositionDegrees;
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public void stop() {
    goalSeekingEnabled = false;
    openLoopEnabled = false;
    openLoopPercent = 0.0;
    goalDegrees = inputs.absolutePositionDegrees;
    io.stop();
  }
}
