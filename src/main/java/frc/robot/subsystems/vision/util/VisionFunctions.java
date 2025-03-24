package frc.robot.subsystems.vision.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class VisionFunctions {
  public static Pose3d calculateMultiTagResult(
      MultiTargetPNPResult result, Transform3d robotToCameraTransform) {
    Transform3d fieldToCamera = result.estimatedPose.best;
    Pose3d estimatedPose =
        new Pose3d()
            .plus(fieldToCamera)
            .relativeTo(AprilTagFieldLayout.loadField(VisionConstants.field).getOrigin())
            .plus(robotToCameraTransform.inverse());
    return estimatedPose;
  }

  public static Matrix<N3, N1> getStdDevs(
      PhotonCamera camera, Pose2d estimatedPose, PhotonPoseEstimator poseEstimator) {
    List<PhotonTrackedTarget> targets = camera.getAllUnreadResults().get(0).getTargets();
    int numTags = 0;
    double dist = 0;
    for (PhotonTrackedTarget target : targets) {
      Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.fiducialId);
      if (tagPose.isEmpty()) continue;

      numTags++;

      // Looks weird, but this is only used when there is one tag, so it doesn't need to be
      // averaged.
      dist = tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0) return VisionConstants.singleTagStdDevs;

    if (numTags > 1) return VisionConstants.multiTagStdDevs;

    if (dist > 4) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    return VisionConstants.singleTagStdDevs.times(1 + (dist * dist / 30));
  }
}
