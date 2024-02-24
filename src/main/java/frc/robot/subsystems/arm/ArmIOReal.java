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
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class ArmIOReal implements ArmIO {
  private CANSparkMax leader;
  private CANSparkMax follower;
  private AbsoluteEncoder armEncoder;
  private SparkPIDController armPIDController;
  private double feedforward;
  private Solenoid piston;
  private TrapezoidProfile motorProfile;
  private TrapezoidProfile.State previousState;
  private TrapezoidProfile.State targetState;
  private Timer timer;

  public ArmIOReal() {
    System.out.println("[Init] Creating ArmIOReal");

    leader = new CANSparkMax(kArmLeaderCanId, MotorType.kBrushless);
    armEncoder = leader.getAbsoluteEncoder(Type.kDutyCycle);

    leader.setInverted(false); // TODO: check direction
    leader.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    leader.enableSoftLimit(SoftLimitDirection.kForward, true);
    leader.setSoftLimit(SoftLimitDirection.kForward, (float) SOFT_LIMIT_FORWARD);
    leader.setIdleMode(IdleMode.kBrake);
    armEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
    armEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    armPIDController = leader.getPIDController();
    armPIDController.setP(ARM_DEFAULT_P);
    armPIDController.setI(ARM_DEFAULT_I);
    armPIDController.setD(ARM_DEFAULT_D);
    leader.burnFlash();

    follower.follow(leader);
    follower.burnFlash();

    piston = new Solenoid(PneumaticsModuleType.CTREPCM, ARM_SOLENOID_CHANNEL);

    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrapezoidProfile.html
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
    motorProfile = new TrapezoidProfile(ARM_MOTION_CONSTRAINTS);
    previousState = new TrapezoidProfile.State(armEncoder.getPosition(), 0.0);
    targetState = previousState;

    timer = new Timer();
    timer.start();
    timer.reset();
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle = armEncoder.getPosition();
    inputs.isExtended = piston.get(); // TODO: check that default is what we think
  }

  /** Manuel input for the arm */
  public void manuel(double manuelInput) {
    previousState = new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
    targetState = new TrapezoidProfile.State(previousState.position, 0.0);
    timer.reset();

    feedforward = ARM_FF.calculate(armEncoder.getPosition(), armEncoder.getVelocity());

    leader.set((manuelInput * ARM_MANUAL_SCALED) + (feedforward / 12.0));
  }

  /** Go to position control for the arm */
  public void goToPos(double targetPosition) {
    targetState = new TrapezoidProfile.State(targetPosition, 0.0);
    previousState = motorProfile.calculate(timer.get(), previousState, targetState);
    timer.reset();

    feedforward = ARM_FF.calculate(armEncoder.getPosition(), armEncoder.getVelocity());
    // set the arm motor speed to the target position
    armPIDController.setReference(
        targetState.position,
        CANSparkMax.ControlType.kPosition,
        0,
        feedforward,
        ArbFFUnits.kVoltage);
  }

  /** Retracts Pistons */
  public void retract() {
    piston.set(false); // TODO: confirm
  }

  /** Extends Pistons */
  public void extend() {
    piston.set(true); // TODO: confirm
  }
}
