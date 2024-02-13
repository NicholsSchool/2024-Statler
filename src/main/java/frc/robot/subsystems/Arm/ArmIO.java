package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double angle = 0.0;
    public boolean isExtended = false;
  }
  /** Updates the set of loggable inputs. */
  public void updateInputs(ArmIOInputs inputs);

  /** Stops all motors */
  public void stop();

  /** Enable or disable brake mode on the motors. */
  public void setBrakeMode(boolean brake);
}
