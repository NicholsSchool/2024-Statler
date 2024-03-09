package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class ClimbIOReal implements ClimbIO {

  private CANSparkMax leftClimb;
  private CANSparkMax rightClimb;

  private RelativeEncoder leftClimbEncoder;
  private RelativeEncoder rightClimbEncoder;

  private Solenoid climbLock;

  public ClimbIOReal() {
    System.out.println("[Init] Creating ClimbIOReal");

    leftClimb = new CANSparkMax(Constants.CAN.kLeftClimberId, MotorType.kBrushless);
    rightClimb = new CANSparkMax(Constants.CAN.kRightClimberId, MotorType.kBrushless);

    leftClimbEncoder = leftClimb.getEncoder();
    rightClimbEncoder = rightClimb.getEncoder();

    leftClimb.setIdleMode(IdleMode.kBrake);
    leftClimb.setInverted(false);
    leftClimb.setSmartCurrentLimit(35); // amps
    leftClimbEncoder.setPositionConversionFactor(2 * Math.PI / 25.0);
    leftClimbEncoder.setVelocityConversionFactor(2 * Math.PI / 25.0);
    leftClimb.burnFlash();

    rightClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.setInverted(true);
    rightClimb.setSmartCurrentLimit(35); // amps
    rightClimbEncoder.setPositionConversionFactor(2 * Math.PI / 25.0);
    rightClimbEncoder.setVelocityConversionFactor(2 * Math.PI / 25.0);
    rightClimb.burnFlash();

    climbLock =
        new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClimbConstants.CLIMB_SOLENOID_CHANNEL);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftPositionRads = leftClimbEncoder.getPosition();
    inputs.rightPositionRads = rightClimbEncoder.getPosition();
    inputs.leftVelocityRadsPerSec = leftClimbEncoder.getVelocity();
    inputs.rightVelocityRadsPerSec = rightClimbEncoder.getVelocity();
    inputs.leftClimbCurrent = leftClimb.getOutputCurrent();
    inputs.rightClimbCurrent = rightClimb.getOutputCurrent();
    inputs.isLocked = isLocked();
  }

  public void setVoltageLeft(double voltage) {
    if (this.isLocked()) leftClimb.setVoltage(0.0);
    else leftClimb.setVoltage(voltage);
  }

  public void setVoltageRight(double voltage) {
    if (this.isLocked()) rightClimb.setVoltage(0.0);
    else rightClimb.setVoltage(voltage);
  }

  public void lock() {
    climbLock.set(true);
  }

  public void unlock() {
    climbLock.set(false);
  }

  public boolean isLocked() {
    return !climbLock.get();
  }

  @Override
  public void stop() {}
}
