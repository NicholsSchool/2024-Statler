package frc.robot.subsystems.hand;

import static frc.robot.Constants.CAN.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class HandIOReal implements HandIO {
  private TalonSRX frontLeftMotor;
  private TalonSRX backLeftMotor;
  private TalonSRX frontRightMotor;
  private TalonSRX backRightMotor;

  public HandIOReal() {
    System.out.println("[Init] Creating HandIOReal");

    frontLeftMotor = new TalonSRX(kFrontLeftShooter);
    frontRightMotor = new TalonSRX(kFrontRightShooter);
    backLeftMotor = new TalonSRX(kBackLeftShooter);
    backRightMotor = new TalonSRX(kBackRightShooter);

    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);

    TalonSRXConfiguration frontConfig = new TalonSRXConfiguration();
    frontConfig.peakCurrentLimit = 35;
    frontConfig.peakCurrentDuration = 500;
    frontConfig.continuousCurrentLimit = 35;

    TalonSRXConfiguration backConfig = new TalonSRXConfiguration();
    backConfig.peakCurrentLimit = 25;
    backConfig.peakCurrentDuration = 500;
    backConfig.continuousCurrentLimit = 25;

    frontLeftMotor.configAllSettings(frontConfig);
    frontRightMotor.configAllSettings(frontConfig);
    backLeftMotor.configAllSettings(backConfig);
    backRightMotor.configAllSettings(backConfig);
  }

  @Override
  public void updateInputs(HandIOInputs inputs) {
    inputs.appliedVolts =
        new double[] {
          frontLeftMotor.getMotorOutputPercent() * frontLeftMotor.getBusVoltage(),
          frontRightMotor.getMotorOutputPercent() * frontRightMotor.getBusVoltage(),
          backLeftMotor.getMotorOutputPercent() * backLeftMotor.getBusVoltage(),
          backRightMotor.getMotorOutputPercent() * backRightMotor.getBusVoltage()
        };

    inputs.currentAmps =
        new double[] {
          frontLeftMotor.getStatorCurrent(),
          frontRightMotor.getStatorCurrent(),
          backLeftMotor.getStatorCurrent(),
          backRightMotor.getStatorCurrent()
        };
  }

  @Override
  public void setVoltage(double frontVoltage, double backVoltage) {
    frontLeftMotor.set(
        TalonSRXControlMode.PercentOutput, frontVoltage / frontLeftMotor.getBusVoltage());
    frontRightMotor.set(
        TalonSRXControlMode.PercentOutput, frontVoltage / frontRightMotor.getBusVoltage());
    backLeftMotor.set(
        TalonSRXControlMode.PercentOutput, backVoltage / backLeftMotor.getBusVoltage());
    backRightMotor.set(
        TalonSRXControlMode.PercentOutput, backVoltage / backRightMotor.getBusVoltage());
  }
}
