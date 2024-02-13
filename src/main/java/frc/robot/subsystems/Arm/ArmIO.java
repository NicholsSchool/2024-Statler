package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double velocityRPMs = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Set the motor voltage */
  public default void setVoltage(double volts) {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean brake) {}
}
