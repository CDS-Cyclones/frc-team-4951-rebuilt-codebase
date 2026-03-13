package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double leftSetpointRPM = 0.0;
  private double rightSetpointRPM = 0.0;

  public void setVelocityRPM(double rpm) {
    setVelocityRPM(rpm, rpm);
  }

  public void setVelocityRPM(double leftRPM, double rightRPM) {
    leftSetpointRPM = leftRPM;
    rightSetpointRPM = rightRPM;
    io.setVelocityRPM(leftRPM, rightRPM);
  }

  public boolean isAtSpeed() {
    double tol = Constants.ShooterConstants.kVelocityToleranceRPM;
    return Math.abs(inputs.leftVelocityRPM - leftSetpointRPM) < tol
        && Math.abs(inputs.rightVelocityRPM - rightSetpointRPM) < tol;
  }

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void setPower(double power) {
    io.setPower(power);
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
