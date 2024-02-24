package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmGoToPos extends Command {
  private Arm arm;

  public ArmGoToPos(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    arm.setGoToPos();
  }
}
