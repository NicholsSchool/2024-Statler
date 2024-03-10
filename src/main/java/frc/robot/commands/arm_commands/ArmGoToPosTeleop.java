package frc.robot.commands.arm_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmGoToPosTeleop extends Command {
  private Arm arm;

  public ArmGoToPosTeleop(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    System.out.println("executing go to pos");
    arm.setGoToPos();
  }
}
