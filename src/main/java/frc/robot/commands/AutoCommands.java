package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.FieldConstants.StagingLocations;
import frc.robot.commands.arm_commands.ArmGoToPosAuto;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public class AutoCommands {
  // Subsystems
  private final Drive drive;
  private final Arm arm;
  private final Intake intake;

  public AutoCommands(Drive drive, Arm arm, Intake intake) {
    this.drive = drive;
    this.arm = arm;
    this.intake = intake;
  }

  public Command driveToPose(Pose2d pose) {
    var drvToPose =
        new DriveToPose(
            this.drive,
            () -> {
              return AllianceFlipUtil.apply(pose);
            });
    return drvToPose.until(drvToPose::atGoal);
  }

  public Command driveToPoseRelative(Pose2d pose) {
    var drvToPose =
        new DriveToPose(
            this.drive,
            () -> {
              return pose;
            });
    return drvToPose.until(drvToPose::atGoal);
  }

  public Command scoreAmpRelativeBlue() {
    var relativeDriveCommand =
        driveToPoseRelative(
                new Pose2d(Units.inchesToMeters(60.0), 0.0, new Rotation2d(Math.toRadians(270.0))))
            .withTimeout(4.0);
    var raiseArmCommand = new ArmGoToPosAuto(arm, ArmConstants.armAmpPosDeg).withTimeout(3.0);
    var adjustCommand =
        driveToPoseRelative(
                new Pose2d(Units.inchesToMeters(60.0), 0.25, new Rotation2d(Math.toRadians(270.0))))
            .withTimeout(2.0);
    var poopCommand = intake.runPoopCommand().withTimeout(1.0);
    var lowerArmCommand = new ArmGoToPosAuto(arm, ArmConstants.armDrivePosDeg).withTimeout(3.0);
    var crossLineCommand =
        driveToPoseRelative(
                new Pose2d(Units.inchesToMeters(120.0), -0.25, new Rotation2d(Math.toRadians(0.0))))
            .withTimeout(4.0);

    return new SequentialCommandGroup(
        relativeDriveCommand,
        raiseArmCommand,
        adjustCommand,
        poopCommand,
        lowerArmCommand,
        crossLineCommand);
  }

  public Command scoreAmpRelativeRed() {
    var relativeDriveCommand =
        driveToPoseRelative(
                new Pose2d(Units.inchesToMeters(60.0), 0.0, new Rotation2d(Math.toRadians(90.0))))
            .withTimeout(4.0);
    var raiseArmCommand = new ArmGoToPosAuto(arm, ArmConstants.armAmpPosDeg).withTimeout(3.0);
    var adjustCommand =
        driveToPoseRelative(
                new Pose2d(Units.inchesToMeters(60.0), -0.25, new Rotation2d(Math.toRadians(90.0))))
            .withTimeout(2.0);
    var poopCommand = intake.runPoopCommand().withTimeout(1.0);
    var lowerArmCommand = new ArmGoToPosAuto(arm, ArmConstants.armDrivePosDeg).withTimeout(3.0);
    var crossLineCommand =
        driveToPoseRelative(
                new Pose2d(Units.inchesToMeters(120.0), 0.25, new Rotation2d(Math.toRadians(0.0))))
            .withTimeout(4.0);

    return new SequentialCommandGroup(
        relativeDriveCommand,
        raiseArmCommand,
        adjustCommand,
        poopCommand,
        lowerArmCommand,
        crossLineCommand);
  }

  public Command scoreAmpRelativeRedThenStop() {
    var relativeDriveCommand =
        driveToPoseRelative(
                new Pose2d(Units.inchesToMeters(60.0), 0.0, new Rotation2d(Math.toRadians(90.0))))
            .withTimeout(4.0);
    var raiseArmCommand = new ArmGoToPosAuto(arm, ArmConstants.armAmpPosDeg).withTimeout(3.0);
    var adjustCommand =
        driveToPoseRelative(
                new Pose2d(Units.inchesToMeters(60.0), -0.25, new Rotation2d(Math.toRadians(90.0))))
            .withTimeout(2.0);
    var poopCommand = intake.runPoopCommand().withTimeout(1.0);
    var lowerArmCommand = new ArmGoToPosAuto(arm, ArmConstants.armDrivePosDeg).withTimeout(3.0);

    return new SequentialCommandGroup(
        relativeDriveCommand, raiseArmCommand, adjustCommand, poopCommand, lowerArmCommand);
  }

  public Command autoTest() {
    return new AutoFieldTest(drive);
  }

  public Command driveToNote() {
    return new DriveToNote(drive);
  }

  public Command driveNotePickup() {
    return new DriveNotePickup(drive);
  }

  /** EXAMPLE!!!: Scores three game pieces on field-side (high cone, high cube, mid cube). */
  public Command amplifierScoreFour() {
    /*  return driveToPose(
    new Pose2d(FieldConstants.amplifierTranslation, Rotation2d.fromDegrees(180.0)));*/

    return sequence(
        // reset(startingPose),
        new DriveToAmplifier(drive),
        driveToPose(
            AllianceFlipUtil.apply(
                new Pose2d(StagingLocations.spikeTranslations[0], Rotation2d.fromDegrees(0.0)))),
        new DriveToAmplifier(drive),
        driveToPose(
            AllianceFlipUtil.apply(
                new Pose2d(StagingLocations.spikeTranslations[1], Rotation2d.fromDegrees(0.0)))),
        new DriveToAmplifier(drive),
        driveToPose(
            AllianceFlipUtil.apply(
                new Pose2d(StagingLocations.spikeTranslations[2], Rotation2d.fromDegrees(0.0)))),
        new DriveToAmplifier(drive));
  }
}
