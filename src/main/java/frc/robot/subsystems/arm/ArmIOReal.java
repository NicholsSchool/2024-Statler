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
    leader.restoreFactoryDefaults();
    armEncoder = leader.getAbsoluteEncoder(Type.kDutyCycle);

    leader.setInverted(false);
    leader.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    leader.enableSoftLimit(SoftLimitDirection.kForward, true);
    leader.setSoftLimit(SoftLimitDirection.kForward, (float) SOFT_LIMIT_FORWARD);
    leader.setIdleMode(IdleMode.kBrake);
    armEncoder.setPositionConversionFactor(2.0 * Math.PI);
    armEncoder.setVelocityConversionFactor(2.0 * Math.PI);
    // armPIDController = leader.getPIDController();
    // armPIDController.setP(ARM_DEFAULT_P);
    // armPIDController.setI(ARM_DEFAULT_I);
    // armPIDController.setD(ARM_DEFAULT_D);
    leader.burnFlash();

    follower = new CANSparkMax(kArmFollowerCanId, MotorType.kBrushless);
    follower.restoreFactoryDefaults();
    follower.follow(leader);
    follower.burnFlash();

    piston = new Solenoid(PneumaticsModuleType.CTREPCM, ARM_SOLENOID_CHANNEL);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle = armEncoder.getPosition();
    inputs.velocity = armEncoder.getVelocity();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();

    inputs.isExtended = piston.get(); // TODO: check that default is what we think
  }

  @Override
  public void setVoltage(double voltage) {
    System.out.println("setVoltage:  " + voltage);
    leader.setVoltage(voltage);
  }

  /** Retracts Pistons */
  @Override
  public void retract() {
    piston.set(false); // TODO: confirm
  }

  /** Extends Pistons */
  @Override
  public void extend() {
    piston.set(true); // TODO: confirm
  }
}
