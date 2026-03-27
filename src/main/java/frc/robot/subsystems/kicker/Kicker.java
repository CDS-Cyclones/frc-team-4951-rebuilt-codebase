package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private static final double kActiveThreshold = 1e-3;

  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private boolean active = false;

  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
    Logger.recordOutput("Kicker/IsActive", active);
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
}
