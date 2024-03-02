package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {

  public IntakeCommand(Intake intake) {
    intake.runEatCommand();
  }
}
