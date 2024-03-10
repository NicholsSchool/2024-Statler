package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double manuelInput;

  private TrapezoidProfile.State initialState = new TrapezoidProfile.State();
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private ArmFeedforward ARM_FF = new ArmFeedforward(ARM_FF_KS, ARM_FF_KG, ARM_FF_KV, ARM_FF_KA);

  private final ProfiledPIDController armPidController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  private Timer timerMoveToPose; // timer to check for timeout of move to position
  private double voltageCommand = 0.0;

  private double previousVelocity = 0.0;
  double acclerationRad = 0.0;

  private double targetAngle = 0.0;

  // tunable parameters
  private static final LoggedTunableNumber armKg = new LoggedTunableNumber("Arm/kG");
  private static final LoggedTunableNumber armKv = new LoggedTunableNumber("Arm/kV");
  private static final LoggedTunableNumber armKa = new LoggedTunableNumber("Arm/kA");

  private static final LoggedTunableNumber positionToleranceDeg =
      new LoggedTunableNumber("Arm/PositionToleranceDeg");
  // private static final LoggedTunableNumber armMaxVelocityRad =
  //     new LoggedTunableNumber("Arm/MaxVelocityRad");
  // private static final LoggedTunableNumber armMaxAccelerationRad =
  //     new LoggedTunableNumber("Arm/MaxAccelerationRad");
  // private static final LoggedTunableNumber armKp = new LoggedTunableNumber("Arm/Kp");
  // private static final LoggedTunableNumber armKi = new LoggedTunableNumber("Arm/Ki");
  // private static final LoggedTunableNumber armKd = new LoggedTunableNumber("Arm/Kd");
  private static final LoggedTunableNumber moveToPosTimeoutSec =
      new LoggedTunableNumber("Arm/TimeoutSec");

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

    timerMoveToPose = new Timer();
    timerMoveToPose.start();
    timerMoveToPose.reset();

    armKg.initDefault(ARM_FF_KG);
    armKv.initDefault(ARM_FF_KV);
    armKa.initDefault(ARM_FF_KA);
    positionToleranceDeg.initDefault(1.0);
    // armMaxVelocityRad.initDefault(0.85167);
    // armMaxAccelerationRad.initDefault(0.2);
    // armKp.initDefault(3.0);
    // armKi.initDefault(0.0);
    // armKd.initDefault(0.0);
    moveToPosTimeoutSec.initDefault(5.0);

    armPidController.setP(Constants.ArmConstants.ARM_P);
    armPidController.setI(Constants.ArmConstants.ARM_I);
    armPidController.setD(Constants.ArmConstants.ARM_D);
    armPidController.setConstraints(
        new TrapezoidProfile.Constraints(
            Constants.ArmConstants.ARM_VEL_LIMIT, Constants.ArmConstants.ARM_ACCEL_LIMIT));
    armPidController.setTolerance(Units.degreesToRadians(positionToleranceDeg.get()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    updateTunables();

    acclerationRad = (inputs.velocityRadsPerSec - previousVelocity) / 0.02;
    previousVelocity = inputs.velocityRadsPerSec;

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      // manuelInput = 0.0;
      // armState = ArmState.kManuel;
    }

    switch (armState) {
      case kManuel:
        voltageCommand =
            ARM_FF.calculate(
                inputs.angleRads, Constants.ArmConstants.ARM_VEL_LIMIT * softLimit(manuelInput));
        // update pid controller even though not using so that it is informed
        // of latest arm position.
        armPidController.calculate(inputs.angleRads);
        break;
      case kGoToPos:
        voltageCommand =
            armPidController.calculate(inputs.angleRads)
                + ARM_FF.calculate(
                    armPidController.getSetpoint().position,
                    armPidController.getSetpoint().velocity);
        break;
    }
    io.setVoltage(voltageCommand);

    switch (pistonState) {
      case kExtended:
        io.extend();
        break;
      case kRetracted:
        io.retract();
        break;
    }
  }

  // set soft limits on the input velocity of the arm to make
  // sure arm does not extend passed danger position.
  public double softLimit(double inputVel) {
    // weird ranges due to [0, 360] angle range of the arm angle input
    if ((inputs.angleDegs >= 100.0 && inputs.angleDegs <= 200.0) && inputVel > 0
        || (inputs.angleDegs <= 2.0 || inputs.angleDegs >= 200.0) && inputVel < 0) {
      return 0.0;
    }
    return inputVel;
  }

  public boolean hasReachedTarget() {
    if (timerMoveToPose.get() > moveToPosTimeoutSec.get()) {
      System.out.println("Arm Move To Pos Timeout!");
      return true;
    } else {
      boolean reachedGoal = armPidController.atGoal();
      if (reachedGoal) {
        System.out.println("Arm Move To Pos Reached Goal!");
      }
      return reachedGoal;
    }
  }

  // called from run command on every cycle when manual is running.
  public void setManuel(double manuelInput) {
    armState = ArmState.kManuel;
    this.manuelInput = manuelInput;
  }

  // assumption is that this is called once to set the target position, not continuously.
  public void setTargetPos(double targetAngleDeg) {
    targetAngle = targetAngleDeg;

    System.out.println("Arm Go To Pos(deg): " + targetAngleDeg);
    timerMoveToPose.reset();
    initialState = new TrapezoidProfile.State(inputs.angleRads, inputs.velocityRadsPerSec);
    goalState = new TrapezoidProfile.State(Units.degreesToRadians(targetAngleDeg), 0.0);
    // set new goal of the pid controller.
    armPidController.setGoal(Units.degreesToRadians(targetAngleDeg));
  }

  // called from run command on every cycle while motion profile is running.
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

  private void updateTunables() {
    // Update from tunable numbers
    if (armKg.hasChanged(hashCode())
        || armKv.hasChanged(hashCode())
        || armKa.hasChanged(hashCode())) {

      System.out.println("updated arm ff coeffs");
      ARM_FF = new ArmFeedforward(0.0, armKg.get(), armKv.get(), armKa.get());
    }

    // if (armMaxVelocityRad.hasChanged(hashCode())
    //     || armMaxAccelerationRad.hasChanged(hashCode())
    //     || positionToleranceDeg.hasChanged(hashCode())
    //     || armKp.hasChanged(hashCode())
    //     || armKi.hasChanged(hashCode())
    //     || armKd.hasChanged(hashCode())) {
    //   armPidController.setP(armKp.get());
    //   armPidController.setI(armKi.get());
    //   armPidController.setD(armKd.get());
    //   armPidController.setConstraints(
    //       new TrapezoidProfile.Constraints(armMaxVelocityRad.get(),
    // armMaxAccelerationRad.get()));
    //   armPidController.setTolerance(Units.degreesToRadians(positionToleranceDeg.get()));
    // }
  }

  // COMMANDS

  // Create command that will move to target angle until motion completes.
  public Command runGoToPosCommand(double targetAngleDeg) {
    // run eat mode until a note is obtained
    return new FunctionalCommand(
        () -> this.setTargetPos(targetAngleDeg),
        this::setGoToPos,
        interrupted -> this.armState = ArmState.kManuel, // back to manual when done
        this::hasReachedTarget,
        this);
  }

  public double getAngleDeg() {
    return inputs.angleDegs;
  }

  // THINGS TO LOG IN ADV SCOPE

  @AutoLogOutput
  public double getTargetAngle() {
    return targetAngle;
  }

  @AutoLogOutput
  public double getAcceleration() {
    return acclerationRad;
  }

  @AutoLogOutput
  public double getVoltageCommand() {
    return voltageCommand;
  }

  @AutoLogOutput
  public double[] getOutputCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput
  public double getGoalStateDeg() {
    return Units.radiansToDegrees(goalState.position);
  }

  @AutoLogOutput
  public ArmState getMode() {
    return armState;
  }
}
