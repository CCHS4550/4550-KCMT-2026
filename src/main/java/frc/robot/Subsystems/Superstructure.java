package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.FieldConstants;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.WantedState;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.Indexer.IndexerWantedState;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.WantedIntakeState;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterSystemState;
import frc.robot.Subsystems.Shooter.Shooter.ShooterWantedState;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Util.LaunchCalculator;
import frc.robot.Util.ShooterMeasurables;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;

  @AutoLogOutput private WantedSuperstructureState wantedState1 = WantedSuperstructureState.IDLE;
  @AutoLogOutput private SystemState systemState = SystemState.IDLE;

  private ShooterMeasurables shooterCalcs =
      new ShooterMeasurables(false, new Rotation2d(), 0, 0, 0, 0, 0, 0, 0, false);

  public Superstructure(
      SwerveSubsystem swerveSubsystem,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Vision vision) {
    this.swerveSubsystem = swerveSubsystem;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    // Log launching parameters TODO: fix logging bugs later
    var launchCalculator = LaunchCalculator.getInstance();

    shooterCalcs = launchCalculator.getParameters(vision);
    shooter.setShooterMeasurables(shooterCalcs);

    systemState = handleStateTransitions();
    applyStates();

    // Clear launching parameters
    launchCalculator.clearLaunchingParameters();

    Logger.recordOutput(
        "Subsystems/Vision/Shoot Pose",
        new Pose2d(
            vision.getLocation().getX(),
            vision.getLocation().getY(),
            shooterCalcs.getDriveAngle()));
  }

  public void setWantedSuperstructureState(WantedSuperstructureState wantedState) {
    if (!DriverStation.isAutonomous()
        && wantedState != WantedSuperstructureState.SHOOT
        && swerveSubsystem.getSystemState()
            != frc.robot.Subsystems.Drive.SwerveSubsystem.SystemState.DRIVE_TO_POINT) {
      swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE);
    }

    if (DriverStation.isAutonomous()) {
      if (wantedState != WantedSuperstructureState.SHOOT) {
        if (swerveSubsystem.getSystemState()
                != frc.robot.Subsystems.Drive.SwerveSubsystem.SystemState.CHOREO_PATH
            && swerveSubsystem.getSystemState()
                != frc.robot.Subsystems.Drive.SwerveSubsystem.SystemState.DRIVE_TO_POINT) {
          swerveSubsystem.setWantedState(WantedState.IDLE);
        }
      }
    }

    this.wantedState1 = wantedState;
  }

  private void applyStates() {
    switch (systemState) {
      case IDLE:
        if (DriverStation.isDisabled()) {
          swerveSubsystem.setWantedState(WantedState.IDLE);
        }
        intake.setWantedIntakeState(WantedIntakeState.IDLE);
        indexer.setWantedState(IndexerWantedState.IDLE);
        shooter.setWantedState(ShooterWantedState.IDLE);

        break;
      case STOW:
        intake.setWantedIntakeState(WantedIntakeState.STOWED);
        indexer.setWantedState(IndexerWantedState.IDLE);
        if (shooter.getSystemState() != ShooterSystemState.ZERO) {
          shooter.setWantedState(ShooterWantedState.IDLE);
        }
        break;
      case ZERO:
        shooter.setWantedState(ShooterWantedState.ZERO);

        break;

      case INTAKING:
        intake.setWantedIntakeState(WantedIntakeState.INTAKE);
        indexer.setWantedState(IndexerWantedState.IDLE);
        // if (shooter.getSystemState() != ShooterSystemState.ZERO) {
        //   shooter.setWantedState(ShooterWantedState.IDLE);
        // }
        break;

      case SHOOT:
        swerveSubsystem.setTargetRotation(shooterCalcs.getDriveAngle());
        intake.setWantedIntakeState(WantedIntakeState.PUMPING);

        shooter.setWantedState(ShooterWantedState.ACTIVE_SHOOT);

        if (shooter.atSetpoint()) {
          indexer.setWantedState(IndexerWantedState.RUNNING);
        }
        break;

      default:
        if (DriverStation.isDisabled()) {
          swerveSubsystem.setWantedState(WantedState.IDLE);
        }
        intake.setWantedIntakeState(WantedIntakeState.IDLE);
        indexer.setWantedState(IndexerWantedState.IDLE);
        shooter.setWantedState(ShooterWantedState.IDLE);
        break;
    }
  }

  private SystemState handleStateTransitions() {
    if (wantedState1 == WantedSuperstructureState.SHOOT && !shooterCalcs.getIsValid()) {
      return SystemState.IDLE;
    }

    switch (wantedState1) {
      case IDLE:
        return SystemState.IDLE;
      case STOW:
        return SystemState.STOW;
      case ZERO:
        return SystemState.ZERO;

      case INTAKING:
        return SystemState.INTAKING;
      case SHOOT:
        return SystemState.SHOOT;

      default:
        return SystemState.IDLE;
    }
  }

  public enum WantedSuperstructureState {
    IDLE,
    STOW,
    ZERO,
    EXTEND_INTAKE,
    INTAKING,
    SHOOT
  }

  private enum SystemState {
    IDLE,
    STOW,
    ZERO,

    INTAKING,

    SHOOT
  }

  private boolean isPassingZone(double x) {
    return FieldConstants.isBlueAlliance() ? x > 4.75 : x < 11.75;
  }

  private boolean isInsideRectangle(Pose2d pose, Pose2d leftCorner, Pose2d rightCorner) {
    double x = pose.getTranslation().getX();
    double y = pose.getTranslation().getY();

    double minX = Math.min(leftCorner.getTranslation().getX(), rightCorner.getTranslation().getX());
    double maxX = Math.max(leftCorner.getTranslation().getX(), rightCorner.getTranslation().getX());
    double minY = Math.min(leftCorner.getTranslation().getY(), rightCorner.getTranslation().getY());
    double maxY = Math.max(leftCorner.getTranslation().getY(), rightCorner.getTranslation().getY());

    return x >= minX && x <= maxX && y >= minY && y <= maxY;
  }

  private boolean handleTrenchSafety() {
    var pose = Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry();
    if (isInsideRectangle(
            pose, new Pose2d(4, 6.8, new Rotation2d()), new Pose2d(5.2, 8, new Rotation2d()))
        || isInsideRectangle(
            pose, new Pose2d(4, 0.2, new Rotation2d()), new Pose2d(5.2, 1, new Rotation2d()))
        || isInsideRectangle(
            pose, new Pose2d(11.3, 6.8, new Rotation2d()), new Pose2d(12.5, 8, new Rotation2d()))
        || isInsideRectangle(
            pose, new Pose2d(11.3, 0.2, new Rotation2d()), new Pose2d(12.5, 1, new Rotation2d()))) {
      return true;
    }
    return false;
  }
}
