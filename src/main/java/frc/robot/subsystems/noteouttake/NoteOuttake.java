package frc.robot.subsystems.noteouttake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class NoteOuttake extends SubsystemBase {
  private NoteOuttakeIO io;
  private final NoteOuttakeIOInputsAutoLogged inputs = new NoteOuttakeIOInputsAutoLogged();

  private static final LoggedTunableNumber ampVelocity =
      new LoggedTunableNumber("NoteIntake/EatVelocityRPMs");
  private static final LoggedTunableNumber speakerVelocity =
      new LoggedTunableNumber("NoteIntake/VomitVelocityRPMs");
  private static final LoggedTunableNumber trapVelocity =
      new LoggedTunableNumber("NoteIntake/DigestVelocityRPMs");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("NoteIntake/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("NoteIntake/kD");

  private PIDController controller = new PIDController(0.0, 0.0, 0.0);

  private static enum OuttakeMode {
    kStopped,
    kAmp,
    kSpeaker,
    kTrap
  };

  private OuttakeMode mode = OuttakeMode.kStopped;

  // initialize tunable values
  static {
    ampVelocity.initDefault(1000.0);
    speakerVelocity.initDefault(-1000.0);
    trapVelocity.initDefault(500.0);
    kP.initDefault(6.0);
    kD.initDefault(0.0);
  }

  public NoteOuttake(NoteOuttakeIO io) {
    System.out.println("[Init] Creating NoteOuttake");
    this.io = io;
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    double setpoint = 0.0;
    io.updateInputs(inputs);
    Logger.processInputs("NoteOuttake", inputs);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      controller.setP(kP.get());
      controller.setD(kD.get());
    }

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      controller.reset();
      mode = OuttakeMode.kStopped;
    } else {
      switch (mode) {
        case kAmp:
          setpoint = ampVelocity.get();
          break;
        case kSpeaker:
          setpoint = speakerVelocity.get();
          break;
        case kTrap:
          setpoint = trapVelocity.get();
          break;
        case kStopped:
        default:
          setpoint = 0.0;
          break;
      }

      controller.setSetpoint(setpoint);
      io.setVoltage(controller.calculate(inputs.velocityRPMs));
    }
  }

  public void setAmp() {
    mode = OuttakeMode.kAmp;
  }

  public void setSpeaker() {
    mode = OuttakeMode.kSpeaker;
  }

  public void setTrap() {
    mode = OuttakeMode.kTrap;
  }

  public void stop() {
    mode = OuttakeMode.kStopped;
  }
}
