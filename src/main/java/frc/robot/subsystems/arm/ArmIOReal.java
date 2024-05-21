package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.CAN.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmIOReal implements ArmIO {
  private final TalonSRX motorOne;
  private final TalonSRX motorTwo;
  private final AbsoluteEncoder armEncoder;

  public ArmIOReal() {
    System.out.println("[Init] Creating ArmIOReal");

    motorOne = new TalonSRX(kShoulderOneCanId);
    motorTwo = new TalonSRX(kShoulderTwoCanId);

    motorOne.setNeutralMode(NeutralMode.Coast);
    motorTwo.setNeutralMode(NeutralMode.Coast);

    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.peakCurrentLimit = ARM_CURRENT_LIMIT;
    config.peakCurrentDuration = 500;
    config.continuousCurrentLimit = ARM_CURRENT_LIMIT;

    motorOne.configAllSettings(config);
    motorTwo.configAllSettings(config);

    armEncoder = Constants.armEncoder;
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angleRads = checkRangeRadians(armEncoder.getPosition() * 2.0 * Math.PI);
    inputs.angleDegs = checkRangeDegrees(Units.radiansToDegrees(inputs.angleRads));
    inputs.velocityRadsPerSec = armEncoder.getVelocity();
    inputs.appliedOutput =
        new double[] {motorOne.getMotorOutputPercent(), motorTwo.getMotorOutputPercent()};
    inputs.busVoltage = new double[] {motorOne.getBusVoltage(), motorTwo.getBusVoltage()};
    inputs.appliedVolts =
        new double[] {
          inputs.busVoltage[0] * inputs.appliedOutput[0],
          inputs.busVoltage[1] * inputs.appliedOutput[1]
        };
    inputs.currentAmps = new double[] {motorOne.getStatorCurrent(), motorTwo.getStatorCurrent()};
  }

  private double checkRangeRadians(double angle) {
    return angle >= Math.toRadians(200.0) ? angle - 2.0 * Math.PI : angle;
  }

  private double checkRangeDegrees(double angle) {
    return angle >= 200.0 ? angle - 360.0 : angle;
  }

  @Override
  public void setVoltage(double voltage) {
    // motorOne.set(TalonSRXControlMode.PercentOutput, voltage / motorOne.getBusVoltage());
    // motorTwo.set(TalonSRXControlMode.PercentOutput, voltage / motorTwo.getBusVoltage());
  }
}
