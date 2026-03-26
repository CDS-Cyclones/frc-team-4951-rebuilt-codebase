package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double mainSetpointRPM = 0.0;
  private double secondarySetpointRPM = 0.0;

  public void setVelocityRPM(double rpm) {
    setVelocityRPM(rpm, rpm);
  }

  public void setVelocityRPM(double mainRPM, double secondaryRPM) {
    mainSetpointRPM = mainRPM;
    secondarySetpointRPM = secondaryRPM;
    io.setVelocityRPM(mainRPM, secondaryRPM);
  }

  public boolean isAtSpeed() {
    return isMainAtSpeed();
  }

  public boolean isMainAtSpeed() {
    double tol = Constants.ShooterConstants.kVelocityToleranceRPM;
    return Math.abs(inputs.mainVelocityRPM - mainSetpointRPM) < tol;
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
