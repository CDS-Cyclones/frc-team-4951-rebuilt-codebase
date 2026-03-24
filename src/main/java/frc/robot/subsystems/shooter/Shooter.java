package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double mainSetpointRPM = 0.0;
  private double followerSetpointRPM = 0.0;

  public void setVelocityRPM(double rpm) {
    setVelocityRPM(rpm, rpm);
  }

  public void setVelocityRPM(double mainRPM, double followerRPM) {
    mainSetpointRPM = mainRPM;
    followerSetpointRPM = followerRPM;
    io.setVelocityRPM(mainRPM, followerRPM);
  }

  public boolean isAtSpeed() {
    double tol = Constants.ShooterConstants.kVelocityToleranceRPM;
    return Math.abs(inputs.mainVelocityRPM - mainSetpointRPM) < tol
        && Math.abs(inputs.followerVelocityRPM - followerSetpointRPM) < tol;
  }

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void setPower(double power) {
    io.setPower(power);
  }

  public void setFollowerPower(double power) {
    io.setFollowerPower(power);
  }

  public void stop() {
    io.stop();
  }

  public void stopFollower() {
    io.stopFollower();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
