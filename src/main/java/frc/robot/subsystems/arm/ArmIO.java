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
   * sets the arm target position in unknown units
   *
   * @param target the target position in unknown units //TODO: change that
   */
  public default void setTargetPosition(double target) {}

  /**
   * Arm manual override
   *
   * @param inputValue the input value from the joystick axis [-1, 1]
   */
  public default void runManuel(double inputValue) {}

  /** Arm go to position */
  public default void goToPos() {}

  /** Stops all motors */
  public default void stop() {}
}
