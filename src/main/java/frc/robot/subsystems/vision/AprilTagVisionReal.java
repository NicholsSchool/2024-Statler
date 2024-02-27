package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionReal extends PhotonVisionSubsystemBase implements AprilTagVisionIO {

  public AprilTagVisionReal(String cameraName, Transform3d cameraToRobot) {
    super(cameraName, cameraToRobot);
  }

  public void updateInputs(AprilTagVisionIOInputs inputs) {
    if (this.hasTargets()) {
      PhotonTrackedTarget target = this.getBestTarget();
      inputs.estimatedPose = this.getBestPoseEstimate(target);
      inputs.bestID = target.getFiducialId();
      inputs.timestamp = super.getTargetTimestamp();
    }
  }

  public boolean hasTargets() {
    return super.hasTargets();
  }

  /** Please check hasTargets() before this is called. */
  public PhotonTrackedTarget getBestTarget() {
    return super.getBestTarget();
  }

  public Pose3d getBestPoseEstimate(PhotonTrackedTarget target) {
    Pose3d robotPose =
        PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(),
            FieldConstants.aprilTags.getTagPose(target.getFiducialId()).get(),
            super.getCameraToRobotTransform());
    return robotPose;
  }
}
