package frc.robot.subsystems.noteintake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.EffectorTalonConstants.NoteIntakeConstants;

public class NoteIntakeIOReal implements NoteIntakeIO {
  private DigitalInput breamBreak;

  public NoteIntakeIOReal() {
    breamBreak = new DigitalInput(NoteIntakeConstants.kBeamBreakChannel);
    System.out.println("[Init] Creating NoteIntakeIOReal");
  }

  @Override
  public void updateInputs(NoteIntakeIOInputs inputs) {
    inputs.velocityRPMs = 0.0;
    inputs.appliedVolts = 0.0;
    inputs.currentAmps = 0.0;
    inputs.hasNote = !breamBreak.get(); // TODO: might be opposite
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setBrakeMode(boolean brake) {}
}
