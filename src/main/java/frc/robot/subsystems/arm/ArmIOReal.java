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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ArmIOReal implements ArmIO {
  private CANSparkMax leader;
  private CANSparkMax follower;
  private AbsoluteEncoder armEncoder;
  private SparkPIDController armPIDController;
  private Solenoid piston;

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
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle = armEncoder.getPosition();
    inputs.isExtended = piston.get(); // TODO: check that default is what we think
  }

  /** Manuel input for the arm */
  public void manuel(double manuelInput) {
    // TODO: make this
    leader.set(manuelInput);
  }

  /** Go to position control for the arm */
  public void goToPos(double targetPosition) {
    // TODO: make this
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
