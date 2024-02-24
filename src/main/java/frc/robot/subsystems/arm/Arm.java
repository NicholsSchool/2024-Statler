package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  double overrideInput;
  double targetPosition;

  private static enum ArmMode {
    kStopped,
    kManuel,
    kGoToPos
  };

  private static enum PistionMode {
    kExtended,
    kRetracted
  };

  private ArmMode armMode = ArmMode.kStopped;
  private PistionMode pistionMode = PistionMode.kRetracted;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    io.updateProfile();

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.stop();
      armMode = ArmMode.kStopped;
      pistionMode = PistionMode.kRetracted;
    } else {
      switch (armMode) {
        case kGoToPos:
          io.setTargetPosition(targetPosition);
          io.goToPos();
          break;
        case kManuel:
          // TODO: abstracted input or just from the controller?
          // TODO: does this work?
          io.runManuel(overrideInput);
          break;
        case kStopped:
          io.stop();
      }

      switch (pistionMode) {
        case kExtended:
          break;
        case kRetracted:
      }
    }
  }

  public void goToPos(double pos) {
    armMode = ArmMode.kGoToPos;
    targetPosition = pos;
  }

  public void runManuel(double input) {
    armMode = ArmMode.kManuel;
    overrideInput = input;
  }

  public void stop() {
    armMode = ArmMode.kStopped;
  }

  public void extend() {
    pistionMode = PistionMode.kExtended;
  }

  public void retract() {
    pistionMode = PistionMode.kRetracted;
  }
}
