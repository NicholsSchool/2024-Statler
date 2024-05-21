package frc.robot.subsystems.hand;

import org.littletonrobotics.junction.AutoLog;

public interface HandIO {
  @AutoLog
  public static class HandIOInputs {
    public double[] velocityRadPerSec = new double[] {0.0, 0.0, 0.0, 0.0};
    public double[] appliedVolts = new double[] {0.0, 0.0, 0.0, 0.0};
    public double[] currentAmps = new double[] {0.0, 0.0, 0.0, 0.0};
  }
  /** Updates the set of loggable inputs. */
  public void updateInputs(HandIOInputs inputs);

  /** Set the motor voltage */
  public void setVoltage(double frontVolts, double backVolts);
}
