package frc.robot.subsystems.vision.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.struct.StructSerializable;

/** Add your docs here. */
public class VisionResult implements StructSerializable {
  public static VisionResultStruct struct = new VisionResultStruct();

  Pose3d pose;
  double timestamp;
  Matrix<N3, N1> stdDevs;

  public VisionResult(Pose3d pose, double timestamp) {
    this(pose, timestamp, null);
  }

  public VisionResult(Pose3d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    this.pose = pose;
    this.timestamp = timestamp;
    this.stdDevs = stdDevs;
  }

  public Pose3d getPose3d() {
    return pose;
  }

  public Pose2d getPose2d() {
    return pose.toPose2d();
  }

  public double getTimestampSeconds() {
    return timestamp;
  }

  public Matrix<N3, N1> getStdDevs() {
    return stdDevs;
  }
}
