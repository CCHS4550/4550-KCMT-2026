package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.Constants;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIO;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIOInputsAutoLogged;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Util.LaunchCalculator;
import frc.robot.Util.ShooterMeasurables;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private FlywheelIO flywheelIO;
  private ElevationIO elevationIO;

  private Vision vision;

  private LaunchCalculator calculator;

  private ElevationIOInputsAutoLogged elevationInputs = new ElevationIOInputsAutoLogged();
  private FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private boolean atGoal;

  private double indexerStatorCurrent = 0.0;
  private double indexerBasedFeedForward = 0.0;

  private final LinearFilter indexerCurrentFilter =
    LinearFilter.singlePoleIIR(0.1, 0.02);

  private ShooterMeasurables wantedShooterMeasurables;

  public enum ShooterSystemState {
    IDLE,
    GOTO_WANTED_MEASURABLES,
    ZERO
  }

  public enum ShooterWantedState {
    IDLE,
    ACTIVE_SHOOT,
    ZERO
  }

  private ShooterSystemState systemState = ShooterSystemState.IDLE;
  private ShooterWantedState wantedState = ShooterWantedState.IDLE;

  public Shooter(
      ElevationIO elevationIO, FlywheelIO flywheelIO, LaunchCalculator calculator, Vision vision) {
    this.elevationIO = elevationIO;
    this.flywheelIO = flywheelIO;
    this.calculator = calculator;
    this.vision = vision;
    atGoal = false;
  }

  @Override
  public void periodic() {
    // wantedShooterMeasurables = calculator.getParameters(vision);

    // atGoal = atSetpoint();

    flywheelIO.updateInputs(flywheelInputs);
    elevationIO.updateInputs(elevationInputs);

    Logger.processInputs("Subsystems/Shooter/Flywheel", flywheelInputs);
    Logger.processInputs("Subsystems/Shooter/Elevation", elevationInputs);

    // System.out.println("wanted=" + wantedState + " current=" + systemState);

    systemState = handleStateTransitions();
    applyStates();

    // flywheelIO.setVoltage(Voltage.ofBaseUnits(2, Volt));
  }

  public ShooterSystemState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return ShooterSystemState.IDLE;
      case ACTIVE_SHOOT:
        return ShooterSystemState.GOTO_WANTED_MEASURABLES;
      case ZERO:
        return ShooterSystemState.ZERO;
      default:
        return ShooterSystemState.IDLE;
    }
  }

  public void applyStates() {
    switch (systemState) {
      case IDLE:
        elevationIO.setVoltage(0);
        flywheelIO.setVoltage(Voltage.ofBaseUnits(0, Volts));
        break;
      case GOTO_WANTED_MEASURABLES:
        setElevationAngle(Rotation2d.fromRadians(wantedShooterMeasurables.getHoodAngle()));
        setFlywheelSpeed(RadiansPerSecond.of(wantedShooterMeasurables.getFlywheelSpeed()));
        break;
      case ZERO:
        flywheelIO.setVelo(RadiansPerSecond.of(250), calculateIndexerFeedForward());
        // elevationIO.setElevationAngle(new Rotation2d(Degrees.of(70)));
        break;
    }
  }

  private void setElevationAngle(Rotation2d angle) {
    elevationIO.setElevationAngle(angle);
  }

  private void setFlywheelSpeed(AngularVelocity velo) {
    flywheelIO.setVelo(velo, calculateIndexerFeedForward());
  }

  public void setShooterMeasurables(ShooterMeasurables shooterMeasurables) {
    this.wantedShooterMeasurables = shooterMeasurables;
  }

  public void setWantedState(ShooterWantedState wantedState) {
    this.wantedState = wantedState;
  }

  public ShooterSystemState getSystemState() {
    return systemState;
  }

  public void setIndexerStatorCurrent (double current){
    indexerStatorCurrent = current;
  }

  private double calculateIndexerFeedForward() {
    double filteredCurrent =
        indexerCurrentFilter.calculate(indexerStatorCurrent);

    if (filteredCurrent <= Constants.ShooterConstants.INDEXER_CURRENT_THRESHOLD) {
        return 0.0;
    }

    double feedForward =
        (filteredCurrent - Constants.ShooterConstants.INDEXER_CURRENT_THRESHOLD)
            * Constants.ShooterConstants.INDEXER_CURRENT_TO_FF;

    return Math.min(feedForward, Constants.ShooterConstants.INDEXER_FEED_FORWARD_MAX);
}

  public boolean atSetpoint() {
    return MathUtil.isNear(
            flywheelInputs.flywheelVelocityRadPerSec,
            wantedShooterMeasurables.getFlywheelSpeed(),
            wantedShooterMeasurables.getFlywheelSpeed()
                * 0.05) // Allowance is 5% of wanted velocity
        && MathUtil.isNear(
            elevationInputs.elevationAngle.getRadians(),
            wantedShooterMeasurables.getHoodAngle(),
            0.3);
  }
}
