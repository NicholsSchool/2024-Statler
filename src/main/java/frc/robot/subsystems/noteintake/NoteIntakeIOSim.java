package frc.robot.subsystems.noteintake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class NoteIntakeIOSim implements NoteIntakeIO {

  private double appliedVolts = 0.0;
  private FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);

  public NoteIntakeIOSim() {
    System.out.println("[Init] Creating NoteIntakeIOSim");
  }

  @Override
  public void updateInputs(NoteIntakeIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);

    inputs.velocityRPMs = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.hasNote = false;
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }
}
