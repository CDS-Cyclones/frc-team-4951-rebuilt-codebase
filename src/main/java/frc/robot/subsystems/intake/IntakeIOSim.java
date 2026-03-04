// dummy io implementation for simulation purposes
package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;

public class IntakeIOSim implements IntakeIO {

  private double percent = 0.0;
  private boolean wasRunning = false;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = percent * 12.0;
    inputs.currentAmps = 0.0;
    inputs.velocityRPM = percent * 5000.0; // fake number

    boolean isRunning = Math.abs(percent) > 0.01;

    if (RobotBase.isSimulation() && isRunning != wasRunning) {
      System.out.println("Intake " + (isRunning ? "ON" : "OFF"));
      wasRunning = isRunning;
    }
  }

  @Override
  public void setPercent(double percent) {
    this.percent = percent;
  }
}
