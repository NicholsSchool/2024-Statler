package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double manuelInput;
  private double targetPos;

  private static enum ArmState {
    kManuel,
    kGoToPos
  };

  private static enum PistonState {
    kExtended,
    kRetracted
  };

  private ArmState armState = ArmState.kGoToPos;
  private PistonState pistonState = PistonState.kRetracted;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      // TODO: NOTIFY ENTIRE TEAM... safety feature for preventing the arm crashing down on disable
      // hopefully
      armState = ArmState.kGoToPos;
      io.goToPos(targetPos);
    } else {
      switch (armState) {
        case kManuel:
          io.manuel(manuelInput);
          break;
        case kGoToPos:
          io.goToPos(targetPos);
          break;
      }
    }

    switch (pistonState) {
      case kExtended:
        io.extend();
        break;
      case kRetracted:
        io.retract();
        break;
    }
  }

  // called from run command
  public void setManuel(double manuelInput) {
    armState = ArmState.kManuel;
    this.manuelInput = manuelInput;
  }

  public void setTargetPos(double targetPos) {
    this.targetPos = targetPos;
  }

  // called from run command
  public void setGoToPos() {
    armState = ArmState.kGoToPos;
  }

  // called from instant command
  public void setExtended() {
    pistonState = PistonState.kExtended;
  }

  // called from instant command
  public void setRetracted() {
    pistonState = PistonState.kRetracted;
  }
}
