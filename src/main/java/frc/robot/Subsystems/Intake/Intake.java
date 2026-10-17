package frc.robot.Subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final double intakeTolerance = 0.2;

  public enum WantedIntakeState {
    IDLE,
    STOWED,
    INTAKE,
    PUMPING
  }

  public enum SystemState {
    IDLE,
    STOWED,
    INTAKE
  }

  private SystemState systemState = SystemState.STOWED;
  private WantedIntakeState wantedState = WantedIntakeState.STOWED;
  private final IntakeIO intakeIO;

  private Rotation2d desiredIntakeAngle = new Rotation2d(0);

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  private void applyStates() {
    switch (systemState) {
      case STOWED:
        intakeIO.setExtensionVoltage(0.0);

        intakeIO.setExtensionMotorPositionRad(Constants.IntakeConstants.INTAKE_STOWED_RADS, 75, 50);
        break;
      case IDLE:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setSpinnerVoltage(0.0);
        break;
      case INTAKE:
        intakeIO.setSpinnerVoltage(3.0);
        if (!MathUtil.isNear(
            Constants.IntakeConstants.INTAKE_BOTTOM_RADS, inputs.extensionPosRadians, 0.5)) {
          intakeIO.setExtensionMotorPositionRad(
              Constants.IntakeConstants.INTAKE_BOTTOM_RADS, 100, 50);
        }
        break;
      default:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setSpinnerVoltage(0.0);
        break;
    }
  }

  private SystemState handleStateTransitions() {
    switch (wantedState) {
      case STOWED:
        return SystemState.STOWED;
      case IDLE:
        return SystemState.IDLE;
      case INTAKE:
        return SystemState.INTAKE;
      case PUMPING:
        // If we are coming from outside into PUMPING, start with PUMP_DOWN.
        // Otherwise, keep whatever internal pump state we are currently executing.
        return SystemState.STOWED;
      default:
        return SystemState.IDLE;
    }
  }

  public void setWantedIntakeState(WantedIntakeState state) {
    // FIX #2: Idempotency guard. Superstructure calls this every 20ms loop tick.
    // Without this, every call restarts the timer and re-evaluates location before
    // the mechanism has actually moved, immediately skipping timed transitions.
    if (state == this.wantedState) return;
    this.wantedState = state;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Subsystems/Intake", inputs);
    Logger.recordOutput("Subsystems/Intake/SystemState", systemState);
    Logger.recordOutput("Subsystems/Intake/DesiredState", wantedState);
    systemState = handleStateTransitions();
    // System.out.println("Wanted Intake State: "+wantedState);
    // System.out.println("Current Intake State: " + systemState);

    applyStates();
  }
}
