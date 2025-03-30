package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.FieldConstants.Reef;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoAlign {
  public static Pose2d bestLoader(Pose2d currentPose) {
    Pose2d loader =
        AllianceFlipUtil.apply(currentPose).getY() > (FieldConstants.fieldWidth / 2)
            ? AllianceFlipUtil.apply(CoralStation.leftCenterFace)
            : AllianceFlipUtil.apply(CoralStation.rightCenterFace);

    Logger.recordOutput("Drive/BestLoader", loader);
    Logger.recordOutput(
        "Drive/BestLoaderDistance",
        loader.getTranslation().getDistance(currentPose.getTranslation()));

    return loader;
  }

  public static Pose2d bestFace(Pose2d currentPose, double x, double y) {
    int bestIndex = 0;
    double bestScore = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 6; i++) {
      Translation2d faceLocation = AllianceFlipUtil.apply(Reef.centerFaces[i].getTranslation());

      Translation2d robotToFaceVector = faceLocation.minus(currentPose.getTranslation());
      Translation2d driverControlVector = new Translation2d(x, y);

      double faceDistanceScore = robotToFaceVector.getNorm();
      double driverInputScore;
      if (driverControlVector.getNorm() < .1) {
        driverInputScore = 0;
      } else {
        Rotation2d robotToFaceAngle = robotToFaceVector.getAngle();
        Rotation2d driverControlAngle = driverControlVector.getAngle();

        driverInputScore = driverControlAngle.minus(robotToFaceAngle).getCos() * 2;
      }

      Logger.recordOutput("Drive/Reef/Face " + i + "/Distance", faceDistanceScore);
      Logger.recordOutput("Drive/Reef/Face " + i + "/Input", driverInputScore);
      double faceScore = faceDistanceScore - driverInputScore;
      Logger.recordOutput("Drive/Reef/Face " + i + "/Score", faceScore);

      if (faceScore < bestScore) {
        bestIndex = i;
        bestScore = faceScore;
      }
    }
    Pose2d bestFace = AllianceFlipUtil.apply(Reef.centerFaces[bestIndex]);
    Logger.recordOutput("Drive/BestReef", bestFace);
    Logger.recordOutput(
        "Drive/BestReefDistance",
        bestFace.getTranslation().getDistance(currentPose.getTranslation()));

    return bestFace;
  }
}
