package frc.robot.subsystems.hand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.HandConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hand extends SubsystemBase {
  private HandIO io;
  private final HandIOInputsAutoLogged inputs = new HandIOInputsAutoLogged();

  private static final LoggedTunableNumber intakeVelocity =
      new LoggedTunableNumber("Hand/IntakeVelocityRPMs");
  private static final LoggedTunableNumber shootVelocity =
      new LoggedTunableNumber("Hand/ShootVelocityRPMs");

  private final SimpleMotorFeedforward ffModel;

  private static double backSetpointRPMs = 0.0;
  private static double backSetpointRadPerSec = 0.0;

  private static double frontSetpointRPMs = 0.0;
  private static double frontSetpointRadPerSec = 0.0;

  private static enum HandMode {
    kStopped,
    kIntaking,
    kRevving,
    kShooting
  };

  private HandMode mode = HandMode.kStopped;

  public Hand(HandIO io) {
    System.out.println("[Init] Creating Hand");
    this.io = io;

    intakeVelocity.initDefault(Constants.HandConstants.kIntakeRPM);
    shootVelocity.initDefault(Constants.HandConstants.kShootRMP);

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        ffModel = new SimpleMotorFeedforward(0.1, 0.12);
        break;
      case ROBOT_SIM:
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.09);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hand", inputs);

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0, 0.0);
      mode = HandMode.kStopped;
    } else {
      switch (mode) {
        case kIntaking:
          frontSetpointRPMs = intakeVelocity.get();
          backSetpointRPMs = intakeVelocity.get();
          break;
        case kRevving:
          frontSetpointRPMs = shootVelocity.get();
          backSetpointRPMs = 0.0;
          break;
        case kShooting:
          frontSetpointRPMs = shootVelocity.get();
          backSetpointRPMs = shootVelocity.get();
          break;
        case kStopped:
          frontSetpointRPMs = 0.0;
          backSetpointRPMs = 0.0;
      }

      frontSetpointRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(frontSetpointRPMs);
      backSetpointRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(backSetpointRPMs);

      double frontVoltage = ffModel.calculate(frontSetpointRadPerSec);
      double backVoltage = ffModel.calculate(backSetpointRadPerSec);
      io.setVoltage(
          MathUtil.clamp(frontVoltage, -12.0, 12.0), MathUtil.clamp(backVoltage, -12.0, 12.0));
    }
  }

  public void intake() {
    mode = HandMode.kIntaking;
  }

  public void rev() {
    mode = HandMode.kRevving;
  }

  public void shoot() {
    mode = HandMode.kShooting;
  }

  public void stop() {
    mode = HandMode.kStopped;
  }

  @AutoLogOutput
  public double getFrontSetpointRadians() {
    return frontSetpointRadPerSec;
  }

  @AutoLogOutput
  public double getBacktSetpointRadians() {
    return backSetpointRadPerSec;
  }

  @AutoLogOutput
  public double getFrontFF() {
    return ffModel.calculate(frontSetpointRadPerSec);
  }

  @AutoLogOutput
  public double getBackFF() {
    return ffModel.calculate(backSetpointRadPerSec);
  }

  @AutoLogOutput
  public HandMode getState() {
    return mode;
  }

  @AutoLogOutput
  public double getFrontSetPointRPMs() {
    return frontSetpointRPMs;
  }

  @AutoLogOutput
  public double getBackSetPointRPMs() {
    return backSetpointRPMs;
  }

  public Command runShootCommand() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("Hand: Shoot"), this),
            new RunCommand(() -> this.rev(), this).withTimeout(HandConstants.kShootDelay),
            new RunCommand(() -> this.shoot(), this).withTimeout(HandConstants.kShootDelay))
        .finallyDo(() -> this.stop());
  }
}
