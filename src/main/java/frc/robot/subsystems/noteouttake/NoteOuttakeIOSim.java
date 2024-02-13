package frc.robot.subsystems.noteouttake;

public class NoteOuttakeIOSim implements NoteOuttakeIO {

  public NoteOuttakeIOSim() {
    System.out.println("[Init] Creating NoteOuttakeIOSim");
  }

  @Override
  public void updateInputs(NoteOuttakeIOInputs inputs) {
    inputs.velocityRPMs = 0.0;
    inputs.appliedVolts = 0.0;
    inputs.currentAmps = 0.0;
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setBrakeMode(boolean brake) {}
}
