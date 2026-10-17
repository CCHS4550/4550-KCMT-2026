package frc.robot.Subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIO.FlywheelIOInputs;
import frc.robot.Util.Phoenix6Util;

public class FlywheelIOCTRE implements FlywheelIO {
  private TalonFX flywheelMotor1;
  private TalonFX flywheelMotor2;
  private TalonFX flywheelMotor3;
  private TalonFX flywheelMotor4;

  private MotionMagicVelocityVoltage flywheelControl;

  private TalonFXConfiguration shooterConfig;

  private final StatusSignal<Voltage> flywheel1AppliedVoltage;
  private final StatusSignal<Current> flywheel1SupplyCurrent;
  private final StatusSignal<Current> flywheel1StatorCurrent;
  private final StatusSignal<Temperature> flywheel1Temperature;

  private final StatusSignal<Voltage> flywheel2AppliedVoltage;
  private final StatusSignal<Current> flywheel2SupplyCurrent;
  private final StatusSignal<Current> flywheel2StatorCurrent;
  private final StatusSignal<Temperature> flywheel2Temperature;

  private final StatusSignal<Voltage> flywheel3AppliedVoltage;
  private final StatusSignal<Current> flywheel3SupplyCurrent;
  private final StatusSignal<Current> flywheel3StatorCurrent;
  private final StatusSignal<Temperature> flywheel3Temperature;

  private final StatusSignal<Voltage> flywheel4AppliedVoltage;
  private final StatusSignal<Current> flywheel4SupplyCurrent;
  private final StatusSignal<Current> flywheel4StatorCurrent;
  private final StatusSignal<Temperature> flywheel4Temperature;

  private final StatusSignal<AngularVelocity> flywheelVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> flywheelAccelerationRotationsPerSecSquared;

  public FlywheelIOCTRE(BruinRobotConfig config) {
    flywheelMotor1 =
        new TalonFX(config.FLYWHEEL_MOTOR_1.getDeviceNumber(), config.FLYWHEEL_MOTOR_1.getBus());
    flywheelMotor2 =
        new TalonFX(config.FLYWHEEL_MOTOR_2.getDeviceNumber(), config.FLYWHEEL_MOTOR_2.getBus());
    flywheelMotor3 =
        new TalonFX(config.FLYWHEEL_MOTOR_3.getDeviceNumber(), config.FLYWHEEL_MOTOR_3.getBus());
    flywheelMotor4 =
        new TalonFX(config.FLYWHEEL_MOTOR_4.getDeviceNumber(), config.FLYWHEEL_MOTOR_4.getBus());

    Follower followerAligned =
        new Follower(config.FLYWHEEL_MOTOR_1.getDeviceNumber(), MotorAlignmentValue.Aligned);

    Follower followerOpposed =
        new Follower(config.FLYWHEEL_MOTOR_1.getDeviceNumber(), MotorAlignmentValue.Opposed);

    flywheelMotor2.setControl(followerAligned);
    flywheelMotor3.setControl(followerOpposed);
    flywheelMotor4.setControl(followerOpposed);
    shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    shooterConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    shooterConfig.Slot0.kP = config.getShooterConfig().shooterKp;
    shooterConfig.Slot0.kI = config.getShooterConfig().shooterKi;
    shooterConfig.Slot0.kD = config.getShooterConfig().shooterKd;
    shooterConfig.Slot0.kS = config.getShooterConfig().shooterKs;
    shooterConfig.Slot0.kV = config.getShooterConfig().shooterKv;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // change this later!
    shooterConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
    shooterConfig.MotionMagic.MotionMagicAcceleration = 50;

    shooterConfig.MotionMagic.MotionMagicExpo_kV = 8.0;

    Phoenix6Util.applyAndCheckConfiguration(flywheelMotor1, shooterConfig, 5);
    Phoenix6Util.applyAndCheckConfiguration(flywheelMotor2, shooterConfig, 5);

    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    Phoenix6Util.applyAndCheckConfiguration(flywheelMotor3, shooterConfig, 5);
    Phoenix6Util.applyAndCheckConfiguration(flywheelMotor4, shooterConfig, 5);

    flywheelMotor2.setControl(followerAligned);
    flywheelMotor3.setControl(followerOpposed);
    flywheelMotor4.setControl(followerOpposed);

    flywheel1AppliedVoltage = flywheelMotor1.getMotorVoltage();
    flywheel1StatorCurrent = flywheelMotor1.getStatorCurrent();
    flywheel1SupplyCurrent = flywheelMotor1.getSupplyCurrent();
    flywheel1Temperature = flywheelMotor1.getDeviceTemp();

    flywheel2AppliedVoltage = flywheelMotor2.getMotorVoltage();
    flywheel2StatorCurrent = flywheelMotor2.getStatorCurrent();
    flywheel2SupplyCurrent = flywheelMotor2.getSupplyCurrent();
    flywheel2Temperature = flywheelMotor2.getDeviceTemp();

    flywheel3AppliedVoltage = flywheelMotor3.getMotorVoltage();
    flywheel3StatorCurrent = flywheelMotor3.getStatorCurrent();
    flywheel3SupplyCurrent = flywheelMotor3.getSupplyCurrent();
    flywheel3Temperature = flywheelMotor3.getDeviceTemp();

    flywheel4AppliedVoltage = flywheelMotor4.getMotorVoltage();
    flywheel4StatorCurrent = flywheelMotor4.getStatorCurrent();
    flywheel4SupplyCurrent = flywheelMotor4.getSupplyCurrent();
    flywheel4Temperature = flywheelMotor4.getDeviceTemp();

    flywheelVelocityRotationsPerSec = flywheelMotor1.getVelocity();
    flywheelAccelerationRotationsPerSecSquared = flywheelMotor1.getAcceleration();

    flywheelControl = new MotionMagicVelocityVoltage(RadiansPerSecond.of(0));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        flywheel1AppliedVoltage,
        flywheel1StatorCurrent,
        flywheel1SupplyCurrent,
        flywheel1Temperature,
        flywheel2AppliedVoltage,
        flywheel2StatorCurrent,
        flywheel2SupplyCurrent,
        flywheel2Temperature,
        flywheel3AppliedVoltage,
        flywheel3StatorCurrent,
        flywheel3SupplyCurrent,
        flywheel3Temperature,
        flywheel4AppliedVoltage,
        flywheel4StatorCurrent,
        flywheel4SupplyCurrent,
        flywheel4Temperature,
        flywheelVelocityRotationsPerSec,
        flywheelAccelerationRotationsPerSecSquared);

    inputs.flywheel1AppliedVoltage = flywheel1AppliedVoltage.getValueAsDouble();
    inputs.flywheel1StatorCurrent = flywheel1StatorCurrent.getValueAsDouble();
    inputs.flywheel1SupplyCurrent = flywheel1SupplyCurrent.getValueAsDouble();
    inputs.flywheel1Temperature = flywheel1Temperature.getValueAsDouble();

    inputs.flywheel2AppliedVoltage = flywheel2AppliedVoltage.getValueAsDouble();
    inputs.flywheel2StatorCurrent = flywheel2StatorCurrent.getValueAsDouble();
    inputs.flywheel2SupplyCurrent = flywheel2SupplyCurrent.getValueAsDouble();
    inputs.flywheel2Temperature = flywheel2Temperature.getValueAsDouble();

    inputs.flywheel3AppliedVoltage = flywheel3AppliedVoltage.getValueAsDouble();
    inputs.flywheel3StatorCurrent = flywheel3StatorCurrent.getValueAsDouble();
    inputs.flywheel3SupplyCurrent = flywheel3SupplyCurrent.getValueAsDouble();

    inputs.flywheelVelocityRadPerSec =
        Units.rotationsToRadians(flywheelVelocityRotationsPerSec.getValueAsDouble());
    inputs.flywheelAccelRadPerSecPerSec =
        Units.rotationsToRadians(flywheelAccelerationRotationsPerSecSquared.getValueAsDouble());
  }

  @Override
  public void setVoltage(
      Voltage voltage) { // why is this voltage and not just an int, what other unit would this be?
    // TODO: fix
    flywheelMotor1.setVoltage(voltage.magnitude());
  }

  @Override
  public void setVelo(AngularVelocity velo, double FF) {
    flywheelMotor1.setControl(
        flywheelControl.withVelocity(velo).withSlot(0).withEnableFOC(true).withFeedForward(FF));
  }
}
