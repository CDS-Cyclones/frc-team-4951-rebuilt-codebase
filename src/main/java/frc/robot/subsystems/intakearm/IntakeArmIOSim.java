package frc.robot.subsystems.intakearm;

import static frc.robot.Constants.IntakeArmConstants.kDeployedPositionDegrees;
import static frc.robot.Constants.IntakeArmConstants.kStowedPositionDegrees;

import edu.wpi.first.math.MathUtil;

public class IntakeArmIOSim implements IntakeArmIO {
  private static final double kNominalVoltage = 12.0;
  private static final double kResponse = 0.2;

  private double appliedVolts = 0.0;
  private double positionDegrees = kStowedPositionDegrees;
  private double velocityDegreesPerSec = 0.0;

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    double targetVelocity =
        (appliedVolts / kNominalVoltage)
            * frc.robot.Constants.IntakeArmConstants.kMaxVelocityDegreesPerSecond;
    velocityDegreesPerSec += (targetVelocity - velocityDegreesPerSec) * kResponse;
    positionDegrees += velocityDegreesPerSec * 0.02;

    double minPosition = Math.min(kStowedPositionDegrees, kDeployedPositionDegrees);
    double maxPosition = Math.max(kStowedPositionDegrees, kDeployedPositionDegrees);
    if (positionDegrees < minPosition) {
      positionDegrees = minPosition;
      velocityDegreesPerSec = 0.0;
    } else if (positionDegrees > maxPosition) {
      positionDegrees = maxPosition;
      velocityDegreesPerSec = 0.0;
    }

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(appliedVolts) * 2.0;
    inputs.absolutePositionDegrees = positionDegrees;
    inputs.positionDegrees = positionDegrees;
    inputs.velocityDegreesPerSec = velocityDegreesPerSec;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -kNominalVoltage, kNominalVoltage);
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    velocityDegreesPerSec = 0.0;
  }

  @Override
  public void resetPositionDegrees(double positionDegrees) {
    this.positionDegrees = positionDegrees;
    velocityDegreesPerSec = 0.0;
  }
}
