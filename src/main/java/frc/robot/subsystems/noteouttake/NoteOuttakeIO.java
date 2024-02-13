package frc.robot.subsystems.noteouttake;

import org.littletonrobotics.junction.AutoLog;

public interface NoteOuttakeIO {

  @AutoLog
  public static class NoteOuttakeIOInputs {
    public double velocityRPMs = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(NoteOuttakeIOInputs inputs) {}

  /** Set the motor voltage */
  public default void setVoltage(double volts) {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean brake) {}
}
