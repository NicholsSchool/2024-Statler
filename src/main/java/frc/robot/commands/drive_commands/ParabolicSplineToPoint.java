// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class ParabolicSplineToPoint extends Command {
  private final double kP = 1.5;
  private final double allowedError = 0.05;
  private final Translation2d target =
      new Translation2d(Units.inchesToMeters(7 * 12), -Units.inchesToMeters(7 * 12));
  private final Drive drive;

  /** Creates a new ParabolicSplineToPoint. */
  public ParabolicSplineToPoint(Drive drive) {
    this.drive = drive;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    double[] xy = new double[] {2.0 * (target.getX() - pose.getX()), target.getY() - pose.getY()};

    double distance = Math.hypot(xy[0], xy[1]);
    if (distance >= allowedError)
      xy = scaleMagnitude(xy, clip(kP * distance, 0.0, DriveConstants.lowGearScaler));
    else {
      xy = new double[] {0.0, 0.0};
    }

    final double x = xy[0];
    final double y = xy[1];

    drive.runVelocity(new ChassisSpeeds(x, y, DriveCommands.angleToVelocity(0.0, drive.getYaw())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double clip(double input, double min, double max) {
    if (input > max) return max;
    if (input < min) return min;
    return input;
  }

  public double[] scaleMagnitude(double[] input, double newMagnitude) {
    double magnitude = Math.hypot(input[0], input[1]);
    if (magnitude != 0) {
      double ratio = newMagnitude / magnitude;
      return new double[] {input[0] * ratio, input[1] * ratio};
    }
    return new double[] {0.0, 0.0};
  }
}
