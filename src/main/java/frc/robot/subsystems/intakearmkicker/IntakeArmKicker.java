package frc.robot.subsystems.intakearmkicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeArmKicker extends SubsystemBase {
  private static final double kActiveThreshold = 1e-3;

  private final IntakeArmKickerIO io;
  private final IntakeArmKickerIOInputsAutoLogged inputs =
      new IntakeArmKickerIOInputsAutoLogged();
  private boolean active = false;

  public IntakeArmKicker(IntakeArmKickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeArmKicker", inputs);
    Logger.recordOutput("IntakeArmKicker/IsActive", active);
  }

  public void run(double percent) {
    active = Math.abs(percent) > kActiveThreshold;
    io.setPercent(percent);
  }

  public void stop() {
    active = false;
    io.stop();
  }

  public boolean isActive() {
    return active;
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }
}
