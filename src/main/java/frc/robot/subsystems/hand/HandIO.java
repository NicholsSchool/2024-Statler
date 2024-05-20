package frc.robot.subsystems.hand;

import org.littletonrobotics.junction.AutoLog;

public interface HandIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double velocityRadPerSec = 0.0;
    public boolean hasNote = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }
  /** Updates the set of loggable inputs. */
  public void updateInputs(IntakeIOInputs inputs);

  /** Set the motor voltage */
  public void setVoltage(double volts);

  /** Enable or disable brake mode on the motors. */
  public void setBrakeMode(boolean brake);
}
