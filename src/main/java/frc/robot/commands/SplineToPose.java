// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// TJG
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SplineMath;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SplineToPose extends Command {
  private final Drive drive;
  private final boolean slowMode;
  private final Supplier<Pose2d> poseSupplier;
  private SplineMath spline;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;

  private static final LoggedTunableNumber splineKp =
      new LoggedTunableNumber("DriveToPose/DriveKp");
  private static final LoggedTunableNumber splineKd =
      new LoggedTunableNumber("DriveToPose/DriveKd");
  private static final LoggedTunableNumber splineThetaKp =
      new LoggedTunableNumber("DriveToPose/ThetaKp");
  private static final LoggedTunableNumber splineThetaKd =
      new LoggedTunableNumber("DriveToPose/ThetaKd");
  private static final LoggedTunableNumber splineMaxVelocity =
      new LoggedTunableNumber("splineToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber splineMaxVelocitySlow =
      new LoggedTunableNumber("splineToPose/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber splineMaxAcceleration =
      new LoggedTunableNumber("splineToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("splineToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxVelocitySlow =
      new LoggedTunableNumber("splineToPose/ThetaMaxVelocitySlow");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("splineToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber splineTolerance =
      new LoggedTunableNumber("splineToPose/DriveTolerance");
  private static final LoggedTunableNumber splineToleranceSlow =
      new LoggedTunableNumber("splineThetaToPose/DriveToleranceSlow");
  private static final LoggedTunableNumber splineThetaTolerance =
      new LoggedTunableNumber("splineToPose/ThetaTolerance");
  private static final LoggedTunableNumber splineThetaToleranceSlow =
      new LoggedTunableNumber("splineToPose/ThetaToleranceSlow");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("splineToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("splineToPose/FFMinRadius");

  static {
    switch (Constants.getRobot()) {
      case ROBOT_FOOTBALL:
      case ROBOT_REAL:
      case ROBOT_REPLAY:
      case ROBOT_SIM:
        splineKp.initDefault(2.0);
        splineKd.initDefault(0.0);
        splineThetaKp.initDefault(5.0);
        splineThetaKd.initDefault(0.0);
        splineMaxVelocity.initDefault(Units.inchesToMeters(150.0));
        splineMaxVelocitySlow.initDefault(Units.inchesToMeters(50.0));
        splineMaxAcceleration.initDefault(Units.inchesToMeters(95.0));
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxVelocitySlow.initDefault(Units.degreesToRadians(90.0));
        thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
        splineTolerance.initDefault(0.01);
        splineToleranceSlow.initDefault(0.06);
        splineThetaTolerance.initDefault(Units.degreesToRadians(1.0));
        splineThetaToleranceSlow.initDefault(Units.degreesToRadians(3.0));
        ffMinRadius.initDefault(0.2);
        ffMaxRadius.initDefault(0.8);
      default:
        break;
    }
  }

  /** Drives to the specified pose under full software control. */
  public SplineToPose(Drive drive, Pose2d pose) {
    this(drive, false, pose);
  }

  /** Drives to the specified pose under full software control. */
  public SplineToPose(Drive drive, boolean slowMode, Pose2d pose) {
    this(drive, slowMode, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public SplineToPose(Drive drive, Supplier<Pose2d> poseSupplier) {
    this(drive, false, poseSupplier);
  }

  /** Drives to the specified pose under full software control. */
  public SplineToPose(Drive drive, boolean slowMode, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.slowMode = slowMode;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // spline :)
    spline =
        new SplineMath(
            poseSupplier.get().getTranslation(),
            poseSupplier.get().getRotation().getRadians(),
            drive.getPose());
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = drive.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(drive.getFieldVelocity().dx, drive.getFieldVelocity().dy)
                .rotateBy(
                    poseSupplier
                        .get()
                        .getTranslation()
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(currentPose.getRotation().getRadians(), drive.getYawVelocity());
    lastSetpointTranslation = drive.getPose().getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (splineMaxVelocity.hasChanged(hashCode())
        || splineMaxVelocitySlow.hasChanged(hashCode())
        || splineMaxAcceleration.hasChanged(hashCode())
        || splineTolerance.hasChanged(hashCode())
        || splineToleranceSlow.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxVelocitySlow.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || splineThetaTolerance.hasChanged(hashCode())
        || splineThetaToleranceSlow.hasChanged(hashCode())
        || splineKp.hasChanged(hashCode())
        || splineKd.hasChanged(hashCode())
        || splineThetaKp.hasChanged(hashCode())
        || splineThetaKd.hasChanged(hashCode())) {
      driveController.setP(splineKp.get());
      driveController.setD(splineKd.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? splineMaxVelocitySlow.get() : splineMaxVelocity.get(),
              splineMaxAcceleration.get()));
      driveController.setTolerance(slowMode ? splineToleranceSlow.get() : splineTolerance.get());
      thetaController.setP(splineThetaKp.get());
      thetaController.setD(splineThetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? thetaMaxVelocitySlow.get() : thetaMaxVelocity.get(),
              thetaMaxAcceleration.get()));
      thetaController.setTolerance(
          slowMode ? splineThetaToleranceSlow.get() : splineThetaTolerance.get());
    }

    // Get current and target pose
    var currentPose = drive.getPose();
    var targetPose = poseSupplier.get();
    spline.update(currentPose);

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // // Command speeds
    // var driveVelocity =
    //     new Pose2d(
    //             new Translation2d(),
    //             currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
    //         .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
    //         .getTranslation();
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            spline.driveVector().getX(),
            spline.driveVector().getY(),
            thetaVelocity,
            currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "Odometry/DriveToPoseSetpoint",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("Odometry/DriveToPoseGoal", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.stop();
    Logger.recordOutput("Odometry/DriveToPoseSetpoint", new Pose2d());
    Logger.recordOutput("Odometry/DriveToPoseGoal", new Pose2d());
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /**
   * Checks if the robot pose is within the allowed drive and theta tolerances.
   *
   * <p>//TODO: @tom is this in meters
   *
   * @param driveTolerance the finish distance for drive in meters
   * @param thetaTolerance the angle threashold
   */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }

  public boolean isFinished() {
    running =
        this.withinTolerance(
            Constants.AutoConstants.driveFinishThreshold,
            new Rotation2d(Constants.AutoConstants.angleFinishThreshold));
    return running;
  }
}
