package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 5, 0.004);

  private double appliedVolts = 0.0;

  public IntakeIOSim() {
    System.out.println("[Init] Creating IntakeIOSim");
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO: do fancy pretending for hasNote

    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.hasNote = false;
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = voltage;
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setBrakeMode(boolean brake) {}
}
