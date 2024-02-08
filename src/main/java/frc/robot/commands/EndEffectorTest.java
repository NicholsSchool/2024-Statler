// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.end_effector.EndEffector;

public class EndEffectorTest extends Command {
  EndEffector endEffector;
  CommandXboxController controller;

  /** Creates a new EndEffectorManualControl. */
  public EndEffectorTest(EndEffector endEffector, CommandXboxController controller) {
    this.endEffector = endEffector;
    this.controller = controller;
    addRequirements(endEffector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endEffector.runIntakeVolts(controller.getLeftX() * 12.0);
    endEffector.runOuttakeVolts(controller.getRightX() * 12.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.runIntakeVelocity(0.0);
    endEffector.runOuttakeVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
