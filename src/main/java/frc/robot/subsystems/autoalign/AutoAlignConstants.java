package frc.robot.subsystems.autoalign;

import edu.wpi.first.math.util.Units;

public class AutoAlignConstants {
  public static final double maxAngularSpeed = 10.0;
  public static final double maxAngularAccel = 10.0;
  public static final double maxLinearSpeed = 3.0;
  public static final double maxLinearAccel = 4.0;

  public static final double linearTolerance = Units.inchesToMeters(2.0);
  public static final double angularTolerance = Units.degreesToRadians(2.0);
  public static final double velocityTolerance = 0.5;

  public static final double offsetReefKeepOff = -0.05;
}
