package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final double kActiveThreshold = 1e-3;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean active = false;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/IsActive", active);
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

  public int getFuelCount() {
    return io.getFuelCount();
  }

  public boolean hasFuel() {
    return getFuelCount() > 0;
  }

  public boolean consumeFuel() {
    return io.consumeFuel();
  }

  public void resetSimulationState() {
    io.resetSimulationState();
  }
}
