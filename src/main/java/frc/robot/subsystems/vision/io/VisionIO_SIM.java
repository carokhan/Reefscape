package frc.robot.subsystems.vision.io;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class VisionIO_SIM implements VisionIO {
  PhotonCamera[] cameras;
  PhotonCameraSim[] simCameras;
  VisionSystemSim visionSim;
  PhotonPoseEstimator[] poseEstimators;

  VisionIOInputs inputs;

  public VisionIO_SIM() {
    visionSim = new VisionSystemSim("VisionSim");
    visionSim.addAprilTags(AprilTagFieldLayout.loadField(VisionConstants.field));

    SimCameraProperties camProperties = new SimCameraProperties();
    camProperties.setCalibration(320, 320, Rotation2d.fromDegrees(70));
    camProperties.setFPS(90);

    cameras = new PhotonCamera[VisionConstants.CameraIDs.length];
    simCameras = new PhotonCameraSim[cameras.length];
    poseEstimators = new PhotonPoseEstimator[cameras.length];

    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new PhotonCamera(VisionConstants.CameraIDs[i]);
      simCameras[i] = new PhotonCameraSim(cameras[i], camProperties);
      poseEstimators[i] =
          new PhotonPoseEstimator(
              AprilTagFieldLayout.loadField(VisionConstants.field),
              VisionConstants.strategy,
              VisionConstants.CameraTransforms[i]);

      visionSim.addCamera(simCameras[i], poseEstimators[i].getRobotToCameraTransform());
    }

    inputs = new VisionIOInputs();
  }

  @Override
  public void updateInputs() {
    inputs.unreadResults = getUnreadResults();
  }

  @Override
  public VisionResult[] getUnreadResults() {
    ArrayList<VisionResult> results = new ArrayList<VisionResult>();

    for (int i = 0; i < simCameras.length; i++) {
      PhotonCamera camera = simCameras[i].getCamera();
      PhotonPoseEstimator poseEstimator = poseEstimators[i];

      List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();
      for (PhotonPipelineResult pipelineResult : pipelineResults) {
        Optional<EstimatedRobotPose> pose = poseEstimator.update(pipelineResult);

        if (pose.isPresent()) {
          results.add(new VisionResult(pose.get().estimatedPose, pose.get().timestampSeconds));
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
  public void setRobotPose(Pose2d pose) {
    visionSim.update(pose);
  }
}
