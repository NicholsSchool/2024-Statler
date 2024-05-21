package frc.robot.subsystems.hand;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class HandIOSim implements HandIO {
  private FlywheelSim sim = new FlywheelSim(DCMotor.getCIM(1), 1, 0.004);

  private double appliedVolts = 0.0;

  public HandIOSim() {
    System.out.println("[Init] Creating IntakeIOSim");
    // Create filter of velocity to determine which direction motor is going.
    // this is used to simulate a note retrieval
  }

  @Override
  public void updateInputs(HandIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.velocityRadPerSec = new double[] {sim.getAngularVelocityRadPerSec(), 0.0, 0.0, 0.0};
    inputs.appliedVolts = new double[] {appliedVolts, 0.0, 0.0, 0.0};
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps(), 0.0, 0.0, 0.0};
  }

  @Override
  public void setVoltage(double front, double back) {
    appliedVolts = front;
    sim.setInputVoltage(appliedVolts);
  }
}
