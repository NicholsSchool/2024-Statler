package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double angle = 0.0;
    public boolean isExtended = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Manuel input for the arm */
  public default void manuel(double manuelInput) {}

  /** Go to position control for the arm */
  public default void goToPos(double targetPosition) {}

  /** Retracts Pistons */
  public default void retract() {}

  /** Extends Pistons */
  public default void extend() {}
}
