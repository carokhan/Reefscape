package frc.robot.subsystems.vision.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.struct.MatrixStruct;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class VisionResultStruct implements Struct<VisionResult> {
  private MatrixStruct<N3, N1> matrixStruct = Matrix.getStruct(N3.instance, N1.instance);

  @Override
  public Class<VisionResult> getTypeClass() {
    return VisionResult.class;
  }

  @Override
  public String getTypeName() {
    return "VisionResult";
  }

  @Override
  public int getSize() {
    return Pose3d.struct.getSize() + (Double.SIZE / 8) + matrixStruct.getSize();
  }

  @Override
  public String getSchema() {
    return "Pose3d pose;double timestamp;Matrix stdDevs";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Pose3d.struct, matrixStruct};
  }

  @Override
  public VisionResult unpack(ByteBuffer bb) {
    Pose3d pose = Pose3d.struct.unpack(bb);
    Double timestamp = bb.getDouble();
    Matrix<N3, N1> stdDevs = matrixStruct.unpack(bb);
    return new VisionResult(pose, timestamp, stdDevs);
  }

  @Override
  public void pack(ByteBuffer bb, VisionResult value) {
    Pose3d.struct.pack(bb, value.getPose3d());
    bb.putDouble(value.getTimestampSeconds());
    matrixStruct.pack(bb, value.getStdDevs());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
