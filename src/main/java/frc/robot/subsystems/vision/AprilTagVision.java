package frc.robot.subsystems.vision;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class AprilTagVision extends PhotonVisionSubsystemBase {

    public AprilTagVision() {
        super( Constants.RobotConstants.cameraName, Constants.RobotConstants.cameraToRobot );
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public Pose3d getBestPoseEstimate() {
        PhotonTrackedTarget target = super.getBestTarget();
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(), 
            FieldConstants.aprilTags.getTagPose(target.getFiducialId()).get(), 
            super.getCameraToRobotTransform());
        return robotPose;
    }

}
