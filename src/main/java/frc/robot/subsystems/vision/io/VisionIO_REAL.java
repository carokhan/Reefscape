package frc.robot.subsystems.vision.io;

import static frc.robot.subsystems.vision.util.VisionFunctions.getStdDevs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.util.VisionResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class VisionIO_REAL implements VisionIO {
  PhotonCamera[] cameras;
  PhotonPoseEstimator[] poseEstimators;

  VisionIOInputs inputs;

  public VisionIO_REAL() {
    cameras = new PhotonCamera[VisionConstants.CameraIDs.length];
    poseEstimators = new PhotonPoseEstimator[cameras.length];

    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new PhotonCamera(VisionConstants.CameraIDs[i]);
      poseEstimators[i] =
          new PhotonPoseEstimator(
              AprilTagFieldLayout.loadField(VisionConstants.field),
              VisionConstants.strategy,
              VisionConstants.CameraTransforms[i]);
    }

    inputs = new VisionIOInputs();
  }

  @Override
  public void updateInputs() {
    inputs.unreadResults = getUnreadResults();

    // Logger.processInputs("/Vision/Cameras", inputs);
  }

  @Override
  public VisionResult[] getUnreadResults() {
    ArrayList<VisionResult> results = new ArrayList<VisionResult>();

    for (int i = 0; i < cameras.length; i++) {
      PhotonCamera camera = cameras[i];
      PhotonPoseEstimator poseEstimator = poseEstimators[i];

      List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();
      for (PhotonPipelineResult pipelineResult : pipelineResults) {
        Optional<EstimatedRobotPose> pose = poseEstimator.update(pipelineResult);

        if (pose.isPresent()) {
          results.add(
              new VisionResult(
                  pose.get().estimatedPose,
                  pose.get().timestampSeconds,
                  getStdDevs(camera, pose.get().estimatedPose.toPose2d(), poseEstimator)));
        }

        List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
        int[] targetIds = new int[targets.size()];
        for (int j = 0; j < targets.size(); j++) {
          PhotonTrackedTarget target = targets.get(i);

          int id = target.getFiducialId();
          targetIds[i] = id;

          Logger.recordOutput(
              String.format("/Vision/Targets/Target%d/Rotation", id),
              new Rotation3d(target.getSkew(), target.getPitch(), target.getYaw()));
          Logger.recordOutput(
              String.format("/Vision/Targets/Target%d/Transform", id),
              target.getBestCameraToTarget());
        }

        Logger.recordOutput("/Vision/TrackedIDs", targetIds);
      }
    }

    return (VisionResult[]) results.toArray();
  }

  @Override
  public void setRobotPose(Pose2d pose) {}
}
