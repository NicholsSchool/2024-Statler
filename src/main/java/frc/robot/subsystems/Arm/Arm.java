package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;


//TODO: we need to change the code because it is based off the intake code
// The GoToPos and Extend/Pistion Stuff need to change so that they work for the arm

public class Arm extends SubsystemBase {
  private ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static final LoggedTunableNumber eatVelocity =
      new LoggedTunableNumber("Arm/EatVelocityRPMs");
  private static final LoggedTunableNumber vomitVelocity =
      new LoggedTunableNumber("Arm/VomitVelocityRPMs");
  private static final LoggedTunableNumber digestVelocity =
      new LoggedTunableNumber("Arm/DigestVelocityRPMs");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD");

  private PIDController controller = new PIDController(0.0, 0.0, 0.0);

  private static enum ArmMode {
    kStopped,
    kOveride,
    kGoToPos
  };

  private static enum PistionMode {
    kExtended,
    kRetracted
  };

  private ArmMode mode = ArmMode.kStopped;

  // initialize tunable values
  static {
    eatVelocity.initDefault(1000.0);
    vomitVelocity.initDefault(-1000.0);
    digestVelocity.initDefault(500.0);
    kP.initDefault(6.0);
    kD.initDefault(0.0);
  }

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating NoteIntake");
    this.io = io;
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    double setpoint = 0.0;
    io.updateInputs(inputs);
    Logger.processInputs("NoteIntake", inputs);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      controller.setP(kP.get());
      controller.setD(kD.get());
    }

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      controller.reset();
      mode = ArmMode.kStopped;
    } else {
      switch (mode) {
        case kGoToPos:
          setpoint = eatVelocity.get();
          break;
        case kOveride:
          setpoint = vomitVelocity.get();
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

  public void eat() {
    mode = ArmMode.kGoToPos;
  }

  public void vomit() {
    mode = ArmMode.kOveride;
  }

  public void stop() {
    mode = ArmMode.kStopped;
  }
}
