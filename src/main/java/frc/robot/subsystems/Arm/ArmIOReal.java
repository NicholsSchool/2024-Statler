package frc.robot.subsystems.Arm;

public class ArmIOReal implements ArmIO {

  public ArmIOReal() {
    System.out.println("[Init] Creating NoteIntakeIOReal");
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.velocityRPMs = 0.0;
    inputs.appliedVolts = 0.0;
    inputs.currentAmps = 0.0;
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setBrakeMode(boolean brake) {}
}
