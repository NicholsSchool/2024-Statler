package frc.robot.subsystems.noteintake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class NoteIntake extends SubsystemBase {
  private NoteIntakeIO io;
  private final NoteIntakeIOInputsAutoLogged inputs = new NoteIntakeIOInputsAutoLogged();

  private static final LoggedTunableNumber eatVelocity =
      new LoggedTunableNumber("NoteIntake/EatVelocityRPMs");
  private static final LoggedTunableNumber vomitVelocity =
      new LoggedTunableNumber("NoteIntake/VomitVelocityRPMs");
  private static final LoggedTunableNumber digestVelocity =
      new LoggedTunableNumber("NoteIntake/DigestVelocityRPMs");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("NoteIntake/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("NoteIntake/kD");

  private PIDController controller = new PIDController(0.0, 0.0, 0.0);

  private static enum IntakeMode {
    kStopped,
    kEating,
    kVomiting,
    kDigesting
  };

  private IntakeMode mode = IntakeMode.kStopped;

  // initialize tunable values
  static {
    eatVelocity.initDefault(1000.0);
    vomitVelocity.initDefault(-1000.0);
    digestVelocity.initDefault(500.0);
    kP.initDefault(6.0);
    kD.initDefault(0.0);
  }

  public NoteIntake(NoteIntakeIO io) {
    System.out.println("[Init] Creating NoteIntake");
    this.io = io;
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    double setpoint = 0.0;
    io.updateInputs(inputs);
    Logger.processInputs("NoteIntake", inputs);

    // Placeholder: Update tunable numbers

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      mode = IntakeMode.kStopped;
    } else {
      switch (mode) {
        case kEating:
          setpoint = eatVelocity.get();
          break;
        case kVomiting:
          setpoint = vomitVelocity.get();
          break;
        case kDigesting:
          setpoint = digestVelocity.get();
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

  public boolean hasNote() {
    return inputs.hasNote;
  }

  public boolean hasNoNote() {
    return !inputs.hasNote;
  }

  public void eat() {
    mode = IntakeMode.kEating;
  }

  public void vomit() {
    mode = IntakeMode.kVomiting;
  }

  public void digest() {
    mode = IntakeMode.kDigesting;
  }

  public void stop() {
    mode = IntakeMode.kStopped;
  }

  public Command runEatCommand() {
    // Run eat until a note is obtained
    return new StartEndCommand(() -> this.eat(), () -> this.stop(), this).until(this::hasNote);
  }

  public Command runVomitCommand() {
    // Run eat until a note is expelled.
    // TODO: May need more time after note is not detected to keep running motors to fully expel.
    return new StartEndCommand(() -> this.vomit(), () -> this.stop(), this).until(this::hasNoNote);
  }

  public Command runDigestCommand() {
    // TODO: Update this to push note through
    return new StartEndCommand(() -> this.digest(), () -> this.stop(), this);
  }
}
