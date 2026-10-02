package frc.robot.Subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Util.Phoenix6Util;

public class IndexerIOCTRE implements IndexerIO {
  private TalonFX indexerMotor;

  private TalonFX kickerMotor;

  private TalonFXConfiguration indexerConfig;
  private TalonFXConfiguration kickerConfig;

  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerSupplyCurrentAmps;
  private final StatusSignal<Current> indexerStatorCurrentAmps;
  private final StatusSignal<Temperature> indexerMotorTemp;
  private final StatusSignal<AngularVelocity> indexerVelocityRadPerSec;
  private final StatusSignal<AngularAcceleration> indexerAccelradPerSecSquared;

  private final StatusSignal<Voltage> kickerAppliedVolts;
  private final StatusSignal<Current> kickerSupplyCurrentAmps;
  private final StatusSignal<Current> kickerStatorCurrentAmps;
  private final StatusSignal<Temperature> kickerMotorTemp;
  private final StatusSignal<AngularVelocity> kickerVelocityRadPerSec;
  private final StatusSignal<AngularAcceleration> kickerAccelradPerSecSquared;

  public IndexerIOCTRE(BruinRobotConfig bruinRobotConfig) {
    indexerMotor =
        new TalonFX(
            bruinRobotConfig.INDEXER_MOTOR.getDeviceNumber(),
            bruinRobotConfig.INDEXER_MOTOR.getBus());

    indexerConfig = new TalonFXConfiguration();

    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    indexerConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(indexerMotor, indexerConfig, 5);

    indexerMotor.setPosition(0.0);
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerSupplyCurrentAmps = indexerMotor.getSupplyCurrent();
    indexerStatorCurrentAmps = indexerMotor.getStatorCurrent();
    indexerVelocityRadPerSec = indexerMotor.getVelocity();
    indexerAccelradPerSecSquared = indexerMotor.getAcceleration();
    indexerMotorTemp = indexerMotor.getDeviceTemp();

    kickerMotor =
        new TalonFX(
            bruinRobotConfig.KICKER_MOTOR.getDeviceNumber(),
            bruinRobotConfig.KICKER_MOTOR.getBus());

    kickerConfig = new TalonFXConfiguration();

    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    kickerConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(kickerMotor, kickerConfig, 5);

    kickerMotor.setPosition(0.0);
    kickerAppliedVolts = kickerMotor.getMotorVoltage();
    kickerSupplyCurrentAmps = kickerMotor.getSupplyCurrent();
    kickerStatorCurrentAmps = kickerMotor.getStatorCurrent();
    kickerVelocityRadPerSec = kickerMotor.getVelocity();
    kickerAccelradPerSecSquared = kickerMotor.getAcceleration();
    kickerMotorTemp = kickerMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        indexerAppliedVolts,
        indexerSupplyCurrentAmps,
        indexerStatorCurrentAmps,
        indexerVelocityRadPerSec,
        indexerAccelradPerSecSquared,
        indexerMotorTemp);

    BaseStatusSignal.refreshAll(
        kickerAppliedVolts,
        kickerSupplyCurrentAmps,
        kickerStatorCurrentAmps,
        kickerVelocityRadPerSec,
        kickerAccelradPerSecSquared,
        kickerMotorTemp);

    inputs.indexerVoltage = indexerAppliedVolts.getValueAsDouble();
    inputs.indexerSupplyCurrent = indexerSupplyCurrentAmps.getValueAsDouble();
    inputs.indexerStatorCurrent = indexerStatorCurrentAmps.getValueAsDouble();
    inputs.indexerVelocityRadPerSec = indexerVelocityRadPerSec.getValueAsDouble();
    inputs.indexerAccelRadPerSecSquared = indexerAccelradPerSecSquared.getValueAsDouble();
    inputs.indexerTemperature = indexerMotorTemp.getValueAsDouble();

    inputs.kickerVoltage = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerSupplyCurrent = kickerSupplyCurrentAmps.getValueAsDouble();
    inputs.kickerStatorCurrent = kickerStatorCurrentAmps.getValueAsDouble();
    inputs.kickerVelocityRadPerSec = kickerVelocityRadPerSec.getValueAsDouble();
    inputs.kickerAccelRadPerSecSquared =
        kickerAccelradPerSecSquared.getValueAsDouble()
            * 2
            * Math.PI; // TODO: Update this later with a total gear reduction in constants
    inputs.kickerTemperature = kickerMotorTemp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    indexerMotor.setVoltage(voltage);
    kickerMotor.setVoltage(voltage);
  }
}
