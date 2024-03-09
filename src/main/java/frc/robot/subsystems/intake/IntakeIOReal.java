package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private DigitalInput breamBreak;
  private CANSparkMax motor;
  private AbsoluteEncoder encoder;

  public IntakeIOReal() {
    System.out.println("[Init] Creating IntakeIOReal");

    breamBreak = new DigitalInput(IntakeConstants.kBeamBreakChannel);
    motor = new CANSparkMax(CAN.kIntakeCanId, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    motor.setIdleMode(IdleMode.kCoast);
    encoder.setPositionConversionFactor(2.0 * Math.PI);
    encoder.setVelocityConversionFactor(2.0 * Math.PI);
    motor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRadPerSec = encoder.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.hasNote = !breamBreak.get(); // TODO: might be opposite
    System.out.println("beambreak:  " + breamBreak.get());
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
