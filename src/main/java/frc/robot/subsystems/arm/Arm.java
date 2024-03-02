package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double manuelInput;
  private double targetPos;
  private double feedforward;

  private TrapezoidProfile motorProfile;
  private TrapezoidProfile.State currentState;
  private TrapezoidProfile.State targetState;
  private Timer timer;
  private ArmFeedforward ARM_FF = new ArmFeedforward(0.0, 1.63, 1.91, 0.13);

  private static enum ArmState {
    kManuel,
    kGoToPos,
  };

  private static enum PistonState {
    kExtended,
    kRetracted
  };

  private ArmState armState;
  private PistonState pistonState;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;
    armState = ArmState.kManuel;
    pistonState = PistonState.kRetracted;

    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrapezoidProfile.html
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
    motorProfile = new TrapezoidProfile(ARM_MOTION_CONSTRAINTS);
    currentState = new TrapezoidProfile.State(0.0, 0.0);
    targetState = currentState;

    timer = new Timer();
    timer.start();
    timer.reset();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      // manuelInput = 0.0;
      targetPos = inputs.angle;
    }

    switch (armState) {
      case kManuel:
        double maxVelRadPerSecond = 0.5;
        this.feedforward = ARM_FF.calculate(inputs.angle, maxVelRadPerSecond * manuelInput);
        io.setVoltage(this.feedforward);
        break;
      case kGoToPos:
        targetState = new TrapezoidProfile.State(targetPos, inputs.velocity);
        currentState = motorProfile.calculate(timer.get(), currentState, targetState);
        timer.reset();

        feedforward = ARM_FF.calculate(currentState.position, currentState.velocity);
        io.setVoltage(this.feedforward);
        break;
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

  public boolean hasReachedTarget() {
    return motorProfile.isFinished(timer.get());
  }

  // called from run command
  public void setManuel(double manuelInput) {
    System.out.println("setManuel:  " + manuelInput);
    armState = ArmState.kManuel;
    this.manuelInput = manuelInput;
  }

  public void setTargetPosToCurrent() {
    this.targetPos = inputs.angle;
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
