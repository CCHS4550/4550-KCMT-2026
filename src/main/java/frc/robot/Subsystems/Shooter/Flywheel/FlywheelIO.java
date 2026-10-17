package frc.robot.Subsystems.Shooter.Flywheel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public double flywheel1AppliedVoltage = 0.0;
    public double flywheel1StatorCurrent = 0.0;
    public double flywheel1SupplyCurrent = 0.0;
    public double flywheel1Temperature = 0.0;

    public double flywheel2AppliedVoltage = 0.0;
    public double flywheel2StatorCurrent = 0.0;
    public double flywheel2SupplyCurrent = 0.0;
    public double flywheel2Temperature = 0.0;

    public double flywheel3AppliedVoltage = 0.0;
    public double flywheel3StatorCurrent = 0.0;
    public double flywheel3SupplyCurrent = 0.0;
    public double flywheel3Temperature = 0.0;

    public double flywheel4AppliedVoltage = 0.0;
    public double flywheel4StatorCurrent = 0.0;
    public double flywheel4SupplyCurrent = 0.0;
    public double flywheel4Temperature = 0.0;

    public double flywheelVelocityRadPerSec = 0.0;
    public double flywheelAccelRadPerSecPerSec = 0.0;
  }

  default void updateInputs(FlywheelIOInputs io) {}

  default void setVoltage(Voltage voltage) {}

  default void setVelo(AngularVelocity velom, double FF) {}
}
