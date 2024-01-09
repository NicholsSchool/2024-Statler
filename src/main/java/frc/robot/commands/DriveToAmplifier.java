package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class DriveToAmplifier extends DriveToPose {
  public static final Pose2d amplifierPose =
      new Pose2d(FieldConstants.amplifierTranslation, Rotation2d.fromDegrees(-90.0));

  /** Automatically drives to the amplifier. */
  public DriveToAmplifier(Drive drive) {
    super(
        drive,
        () -> {
          return AllianceFlipUtil.apply(amplifierPose);
        });
  }
}
