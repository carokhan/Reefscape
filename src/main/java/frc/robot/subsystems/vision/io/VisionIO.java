package frc.robot.subsystems.vision.io;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.util.VisionResult;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public class VisionIOInputs {
    public VisionResult[] unreadResults;
  }

  public void updateInputs();

  public VisionResult[] getUnreadResults();

  public void setRobotPose(Pose2d presetPose);
}
