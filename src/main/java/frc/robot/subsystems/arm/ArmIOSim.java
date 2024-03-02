package frc.robot.subsystems.arm;

public class ArmIOSim implements ArmIO {
  double voltage = 0.0;

  public ArmIOSim() {
    System.out.println("[Init] Creating ArmIOSim");
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // inputs.angle = armEncoder.getPosition();
    // inputs.velocity = armEncoder.getVelocity();
    inputs.appliedVolts = voltage; // leader.getAppliedOutput() * leader.getBusVoltage();

    // inputs.isExtended = piston.get(); // TODO: check that default is what we think
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }

  /** Retracts Pistons */
  @Override
  public void retract() {}

  /** Extends Pistons */
  @Override
  public void extend() {}
}
