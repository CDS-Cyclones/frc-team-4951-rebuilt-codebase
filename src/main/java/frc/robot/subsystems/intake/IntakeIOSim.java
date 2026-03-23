package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
  private static final double NOMINAL_VOLTAGE = 12.0;

  private final IntakeSimulation intakeSimulation;
  private double appliedPercent = 0.0;

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
    intakeSimulation =
        IntakeSimulation.InTheFrameIntake(
            "Fuel",
            driveTrain,
            Meters.of(Units.inchesToMeters(16.67)),
            IntakeSimulation.IntakeSide.FRONT,
            40);
    intakeSimulation.register();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSimulation.removeObtainedGamePieces(SimulatedArena.getInstance());
    inputs.appliedVolts = appliedPercent * NOMINAL_VOLTAGE;
    inputs.currentAmps = 0.0;
    inputs.velocityRPM = 0.0;
    inputs.fuelCount = intakeSimulation.getGamePiecesAmount();
  }

  @Override
  public int getFuelCount() {
    return intakeSimulation.getGamePiecesAmount();
  }

  @Override
  public boolean consumeFuel() {
    int fuelCount = intakeSimulation.getGamePiecesAmount();
    if (fuelCount <= 0) {
      return false;
    }

    intakeSimulation.setGamePiecesCount(fuelCount - 1);
    return true;
  }

  @Override
  public void resetSimulationState() {
    appliedPercent = 0.0;
    intakeSimulation.stopIntake();
    intakeSimulation.setGamePiecesCount(0);
  }

  @Override
  public void setPercent(double percent) {
    appliedPercent = percent;
    if (Math.abs(percent) > 1e-3) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }
}
