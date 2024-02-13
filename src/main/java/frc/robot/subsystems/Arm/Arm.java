package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static enum ArmMode {
    kStopped,
    kOveride,
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
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.stop();
      armMode = ArmMode.kStopped;
      pistionMode = PistionMode.kRetracted;
    } else {
      switch (armMode) {
        case kGoToPos:
          break;
        case kOveride:
          break;
        case kStopped:
        default:
          break;
      }

      switch (pistionMode) {
        case kExtended:
          break;
        case kRetracted:
        default:
          break;
      }
    }
  }

  public void goToPos() {
    armMode = ArmMode.kGoToPos;
  }

  public void override() {
    armMode = ArmMode.kOveride;
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
