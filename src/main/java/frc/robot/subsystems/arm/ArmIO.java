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

  public default void updateProfile() {}

  /**
   * Arm manual override
   * 
   * @param inputValue the input value from the joystick axis [-1, 1]
   */
  public default void override(double inputValue) {}

  /**
   * Arm go to position
   * 
   * @param targetPosition the target position in unknown units //TODO: change that
   */
  public default void goToPos(double targetPosition) {}

  /** Stops all motors */
  public default void stop() {}
}
