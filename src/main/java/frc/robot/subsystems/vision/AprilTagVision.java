package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.FieldConstants;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVision extends PhotonVisionSubsystemBase {

  public AprilTagVision() {
    super(Constants.VisionConstants.cameraName, Constants.RobotConstants.cameraToRobot);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public PhotonTrackedTarget getBestTarget() {
    return super.getBestTarget();
  }

  public Pose3d getBestPoseEstimate( PhotonTrackedTarget target ) {
    Pose3d robotPose =
        PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(),
            FieldConstants.aprilTags.getTagPose(target.getFiducialId()).get(),
            super.getCameraToRobotTransform());
    return robotPose;
  }

  public Matrix< N3, N1 > getEstimateStdDevs( PhotonTrackedTarget target ) //TODO: check if working and def not working so make work
  {
    Transform3d bestTargetTransform = target.getBestCameraToTarget();
    double distanceToTarget = Math.hypot(bestTargetTransform.getX(), bestTargetTransform.getY() );
    double angleToTarget = -target.getYaw();
    Matrix< N3, N1 > estimateStdDevs = VecBuilder.fill( 
      distanceToTarget * Constants.VisionConstants.xStdDevsScaler, 
      distanceToTarget * Constants.VisionConstants.yStdDevsScaler, 
      angleToTarget * Constants.VisionConstants.angleStdDevsScaler );
    return estimateStdDevs;
  }
}
