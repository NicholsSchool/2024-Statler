package frc.robot.subsystems.noteintake;

public class NoteIntakeIOReal implements NoteIntakeIO {
  public NoteIntakeIOReal() {
    System.out.println("[Init] Creating NoteIntakeIOReal");
  }

  @Override
  public void updateInputs(NoteIntakeIOInputs inputs) {
    inputs.velocityRPMs = 0.0;
    inputs.appliedVolts = 0.0;
    inputs.currentAmps = 0.0;
    inputs.hasNote = false;
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setBrakeMode(boolean brake) {}
}
