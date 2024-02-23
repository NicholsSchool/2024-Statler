package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.CAN.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class ArmIOReal implements ArmIO {
  private CANSparkMax leader;
  private CANSparkMax follower;
  private AbsoluteEncoder armEncoder;
  private SparkPIDController armPIDController;
  private double armSetpoint = 0.0;
  private TrapezoidProfile motorProfile;
  private TrapezoidProfile.State targetState;
  private double feedforward;
  private Timer timer;

  public ArmIOReal() {
    System.out.println("[Init] Creating ArmIOReal");

    leader = new CANSparkMax(kArmLeaderCanId, MotorType.kBrushless);
    armEncoder = leader.getAbsoluteEncoder(Type.kDutyCycle);

    leader.setInverted(false); // TODO make it right
    leader.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    leader.enableSoftLimit(SoftLimitDirection.kForward, true);
    leader.enableSoftLimit(SoftLimitDirection.kReverse, true);
    leader.setSoftLimit(SoftLimitDirection.kForward, (float) SOFT_LIMIT_FORWARD);
    leader.setSoftLimit(SoftLimitDirection.kReverse, (float) SOFT_LIMIT_REVERSE);
    leader.setIdleMode(IdleMode.kBrake);
    armEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
    armEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    armPIDController = leader.getPIDController();
    armPIDController.setP(ARM_DEFAULT_P);
    armPIDController.setI(ARM_DEFAULT_I);
    armPIDController.setD(ARM_DEFAULT_D);
    leader.burnFlash();

    follower = new CANSparkMax(kArmFollowerCanId, MotorType.kBrushless);
    follower.follow(leader);

    timer = new Timer();
    timer.start();
    timer.reset();

    // TODO: look at the absolute encoder 0, direction through REV hardware client

    // Set the starting state of the arm subsystem.
    updateProfile();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle = 0.0;
    inputs.isExtended = false;
  }

  @Override
  public void setTargetPosition(double target) {
    targetState = new TrapezoidProfile.State(target, 0.0);
  }

  @Override
  public void updateProfile() {
    TrapezoidProfile.State state =
        new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(armSetpoint, 0.0);

    // TODO: @tom this is deprecated
    motorProfile = new TrapezoidProfile(ARM_MOTION_CONSTRAINTS, goal, state);
    // TODO: is the below correct?
    // motorProfile.calculate(timer.get(), state, goal);

    timer.reset();
  }

  @Override
  public void override(double powerProportion) {
    // get the current position of the encoder
    armSetpoint = armEncoder.getPosition();
    // create a new target state with the current encoder position and zero velocity
    targetState = new TrapezoidProfile.State(armSetpoint, 0.0);
    // create a new motion profile with the current state as the target state

    // TODO: same thing here
    motorProfile = new TrapezoidProfile(ARM_MOTION_CONSTRAINTS, targetState, targetState);
    // update the feedforward variable with the new target state
    feedforward =
        ARM_FF.calculate(armEncoder.getPosition() + ARM_ZERO_COSINE_OFFSET, targetState.velocity);
    // set the arm motor speed to manual control with scaled power
    leader.set((powerProportion * ARM_MANUAL_SCALED) + (feedforward / 12.0));
  }

  @Override
  public void goToPos() {
    double elapsedTime = timer.get();
    // if motion profile is finished, set the target state to the current position
    if (motorProfile.isFinished(elapsedTime)) {
      targetState = new TrapezoidProfile.State(armSetpoint, 0.0);
    } else {
      targetState = motorProfile.calculate(elapsedTime); // TODO: also deprecated
    }
    // update the feedforward variable with the new target state
    feedforward =
        ARM_FF.calculate(armEncoder.getPosition() + ARM_ZERO_COSINE_OFFSET, targetState.velocity);
    // set the arm motor speed to the target position
    armPIDController.setReference(
        targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
