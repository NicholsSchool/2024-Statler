package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.CAN.*;
import static frc.robot.Constants.ArmConstants.*;

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

    leader.setInverted(false); //TODO make it right
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

    //TODO: look at the absolute encoder 0, direction through REV hardware client

    // Set the starting state of the arm subsystem.
    updateProfile();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle = 0.0;
    inputs.isExtended = false;
  }

  public void updateProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(armSetpoint, 0.0);

    //TODO: @tom this is deprecated, what is the right way to do this?
    motorProfile = new TrapezoidProfile(ARM_MOTION_CONSTRAINTS, goal, state);
    timer.reset();
  }

  public void override(double inputValue) {

  }

  public void goToPos(double targetPosition) {

  }

  @Override
  public void stop() {

  }
}
