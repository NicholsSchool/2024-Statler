// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmManuel extends Command {
  private Arm arm;
  private double manuelInput;
  /** Creates a new ArmManuel. */
  public ArmManuel(Arm arm, double manuelInput) {
    addRequirements(arm);
    this.arm = arm;
    this.manuelInput = manuelInput;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setManuel(manuelInput);
  }
}
