package frc.robot.subsystems.outtake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.commands.WaitCommandTunable;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private double setpoint;
  private OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
  public double voltageCommand;

  private static final LoggedTunableNumber ampVelocity =
      new LoggedTunableNumber("Outtake/AmpVelocityRPMs");
  private static final LoggedTunableNumber speakerVelocity =
      new LoggedTunableNumber("Outtake/SpeakerVelocityRPMs");
  private static final LoggedTunableNumber trapVelocity =
      new LoggedTunableNumber("Outtake/TrapVelocityRPMs");
  private static final LoggedTunableNumber deliverVelocity =
      new LoggedTunableNumber("Outtake/DeliverVelocityRPMs");
  private static final LoggedTunableNumber spinDurationSec =
      new LoggedTunableNumber("Outtake/SpinDurationSec");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Outtake/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Outtake/kD");

  private PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private final SimpleMotorFeedforward ffModel;

  private static enum OuttakeMode {
    kStopped,
    kAmp,
    kSpeaker,
    kTrap,
    kDeliver
  };

  private OuttakeMode mode = OuttakeMode.kStopped;

  public Outtake(OuttakeIO io) {
    System.out.println("[Init] Creating Outtake");
    this.io = io;
    io.setBrakeMode(false);

    ampVelocity.initDefault(600.0);
    speakerVelocity.initDefault(1000.0);
    trapVelocity.initDefault(0.0);
    deliverVelocity.initDefault(1000.0);
    spinDurationSec.initDefault(1.5);
    kP.initDefault(OuttakeConstants.kP);
    kD.initDefault(OuttakeConstants.kD);

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        ffModel = new SimpleMotorFeedforward(0.001, 0.0060);
        break;
      case ROBOT_SIM:
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0055);
        break;
    }
  }

  @Override
  public void periodic() {
    setpoint = 0.0;
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);

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
        case kDeliver:
          setpoint = deliverVelocity.get();
          break;
        case kStopped:
          setpoint = 0.0;
      }

      voltageCommand = ffModel.calculate(setpoint);
      io.setVoltage(MathUtil.clamp(voltageCommand, -12.0, 12.0));
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

  public void setDeliver() {
    mode = OuttakeMode.kDeliver;
  }

  public void stop() {
    mode = OuttakeMode.kStopped;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @AutoLogOutput
  public double getSetpointVelocityRPMs() {
    return setpoint;
  }

  @AutoLogOutput
  public double getVoltageCommand() {
    return voltageCommand;
  }

  @AutoLogOutput
  public double getActualVelocityRPMs() {
    return inputs.velocityRPMs;
  }

  public Command runAmpCommand() {
    // Run eat until a note is expelled.

    // run a sequence that runs poop mode until note is moved off sensor,
    // then keep running motors for a second to keep advancing,
    // then stop.
    return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("Outtake: Amp"), this),
            new InstantCommand(() -> this.setAmp(), this),
            new WaitCommandTunable(
                () -> spinDurationSec.get()), // run a bit more to advance the note
            new InstantCommand(() -> System.out.println("Outtake: Amp Done"), this))
        .finallyDo(() -> this.stop());
  }

  public Command runSpeakerCommand() {
    // Place note in speaker
    return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("Outtake: Speaker"), this),
            new InstantCommand(() -> this.setSpeaker(), this),
            new WaitCommandTunable(
                () -> spinDurationSec.get()), // run a bit more to advance the note
            new InstantCommand(() -> System.out.println("Outtake: Speaker Done"), this))
        .finallyDo(() -> this.stop());
  }

  public Command runDeliverCommand() {
    // Deliver a note
    return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("Outtake: Deliver"), this),
            new InstantCommand(() -> this.setDeliver(), this),
            new WaitCommandTunable(
                () -> spinDurationSec.get()), // run a bit more to advance the note
            new InstantCommand(() -> System.out.println("Outtake: Deliver Done"), this))
        .finallyDo(() -> this.stop());
  }

  public Command runTrapCommand() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("Outtake: Trap"), this),
            new InstantCommand(() -> this.setTrap(), this),
            new WaitCommandTunable(
                () -> spinDurationSec.get()) // run a bit more to advance the note
            )
        .finallyDo(() -> this.stop());
  }
}
