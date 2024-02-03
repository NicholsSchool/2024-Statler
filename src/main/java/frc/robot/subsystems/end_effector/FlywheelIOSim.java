package frc.robot.subsystems.end_effector;

public class FlywheelIOSim implements FlywheelIO {

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {}

  @Override
  public void stop() {}

  @Override
  public void configurePID(double kP, double kI, double kD) {}
}
