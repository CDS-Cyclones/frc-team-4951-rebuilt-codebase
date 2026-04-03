package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;

/** Controls the CANdle LED strip to provide visual state feedback. */
public class LEDSubsystem extends SubsystemBase {

  public enum LEDState {
    LOW_BATTERY,
    DISABLED_ALLIANCE,
    AUTO_RUNNING,
    READY_TO_FIRE,
    SHOOTER_WARMING,
    END_GAME,
    DEFAULT_IDLE
  }

  private static final double LOW_BATTERY_THRESHOLD = 11.0;
  private static final int TOTAL_LED_COUNT = 68; // Adjust to your strip length + 8 onboard
  private static final int CONFIDENCE_BAR_COUNT = 8;
  private static final int MAIN_LED_END = TOTAL_LED_COUNT - CONFIDENCE_BAR_COUNT;
  private static final int CONF_BAR_START = MAIN_LED_END;

  private final CANdle candle;
  private final Shooter shooter;

  // Pre-allocated control objects to avoid GC pressure
  private final SolidColor mainSolid = new SolidColor(0, MAIN_LED_END);

  private LEDState currentState = LEDState.DEFAULT_IDLE;
  private IntSupplier visibleTagCountSupplier = () -> 0;

  public LEDSubsystem(Shooter shooter) {
    this.shooter = shooter;
    this.candle =
        new CANdle(Constants.DriveConstants.candleCanId, Constants.DriveConstants.canBusName);
  }

  public void setVisibleTagCountSupplier(IntSupplier supplier) {
    this.visibleTagCountSupplier = supplier;
  }

  @Override
  public void periodic() {
    LEDState newState = determineState();
    if (newState != currentState) {
      currentState = newState;
      Logger.recordOutput("LED/State", currentState.name());
    }

    applyMainPattern(currentState);
    applyConfidenceBar(visibleTagCountSupplier.getAsInt());
  }

  private LEDState determineState() {
    if (RobotController.getBatteryVoltage() < LOW_BATTERY_THRESHOLD) return LEDState.LOW_BATTERY;
    if (DriverStation.isDisabled()) return LEDState.DISABLED_ALLIANCE;
    if (DriverStation.isAutonomousEnabled()) return LEDState.AUTO_RUNNING;
    if (shooter.isAtSpeed()) return LEDState.READY_TO_FIRE;
    if (shooter.isActive()) return LEDState.SHOOTER_WARMING;
    double matchTime = DriverStation.getMatchTime();
    if (matchTime >= 0 && matchTime <= 30.0 && DriverStation.isTeleopEnabled())
      return LEDState.END_GAME;
    return LEDState.DEFAULT_IDLE;
  }

  private void setControlSafe(SolidColor control) {
    try {
      candle.setControl(control);
    } catch (Exception e) {
      Logger.recordOutput("LED/Error", "CANdle error: " + e.getMessage());
    }
  }

  private void applyMainPattern(LEDState state) {
    double time = Timer.getFPGATimestamp();
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    RGBWColor color =
        switch (state) {
          case LOW_BATTERY -> {
            int b = (int) (127 + 127 * Math.sin(time * 2 * Math.PI));
            yield new RGBWColor(b, 0, 0);
          }
          case DISABLED_ALLIANCE -> isRed ? new RGBWColor(80, 0, 0) : new RGBWColor(0, 0, 80);
          case AUTO_RUNNING -> new RGBWColor(0, 200, 0);
          case READY_TO_FIRE -> new RGBWColor(0, 255, 0);
          case SHOOTER_WARMING -> {
            int b = (int) (80 + 175 * Math.abs(Math.sin(time * 2 * Math.PI)));
            yield new RGBWColor(0, b, 0);
          }
          case END_GAME -> {
            int b = (int) (80 + 175 * Math.abs(Math.sin(time * 1.5 * Math.PI)));
            yield new RGBWColor(b, 0, b);
          }
          case DEFAULT_IDLE -> {
            int b = (int) (30 + 50 * Math.abs(Math.sin(time * 0.5 * Math.PI)));
            yield isRed ? new RGBWColor(b, 0, 0) : new RGBWColor(0, 0, b);
          }
        };

    mainSolid.Color = color;
    setControlSafe(mainSolid);
  }

  /**
   * Renders the confidence bar: last 8 LEDs show pose estimation quality.
   *
   * <ul>
   *   <li>0 tags: all red (odometry only)
   *   <li>1 tag: 2 LEDs yellow
   *   <li>2 tags: 4 LEDs green
   *   <li>3 tags: 6 LEDs bright green
   *   <li>4+ tags: full bar green
   * </ul>
   */
  private void applyConfidenceBar(int visibleTags) {
    RGBWColor barColor;
    int litCount;

    if (visibleTags == 0) {
      barColor = new RGBWColor(80, 0, 0);
      litCount = CONFIDENCE_BAR_COUNT;
    } else if (visibleTags == 1) {
      barColor = new RGBWColor(120, 100, 0);
      litCount = 2;
    } else if (visibleTags == 2) {
      barColor = new RGBWColor(0, 150, 0);
      litCount = 4;
    } else if (visibleTags == 3) {
      barColor = new RGBWColor(0, 255, 0);
      litCount = 6;
    } else {
      barColor = new RGBWColor(0, 255, 0);
      litCount = CONFIDENCE_BAR_COUNT;
    }

    if (litCount > 0) {
      var litSolid = new SolidColor(CONF_BAR_START, CONF_BAR_START + litCount);
      litSolid.Color = barColor;
      setControlSafe(litSolid);
    }

    if (litCount < CONFIDENCE_BAR_COUNT) {
      var darkSolid =
          new SolidColor(CONF_BAR_START + litCount, CONF_BAR_START + CONFIDENCE_BAR_COUNT);
      darkSolid.Color = new RGBWColor(0, 0, 0);
      setControlSafe(darkSolid);
    }

    Logger.recordOutput("LED/ConfidenceTags", visibleTags);
    Logger.recordOutput("LED/ConfidenceBar", litCount + "/" + CONFIDENCE_BAR_COUNT);
  }
}
