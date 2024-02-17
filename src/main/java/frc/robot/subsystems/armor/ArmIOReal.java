package frc.robot.subsystems.armor;

public class ArmIOReal implements ArmIO {

  public ArmIOReal() {
    System.out.println("[Init] Creating ArmIOReal");
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle = 0.0;
    inputs.isExtended = false;
  }

  @Override
  public void stop() {}
}
