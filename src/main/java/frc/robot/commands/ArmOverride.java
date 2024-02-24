package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmOverride extends Command {

  public ArmOverride(Arm arm, double inputValue) {
    addRequirements(arm);

    arm.runManuel(inputValue);
  }
}
