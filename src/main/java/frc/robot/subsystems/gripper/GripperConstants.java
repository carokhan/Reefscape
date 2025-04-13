package frc.robot.subsystems.gripper;

import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;

public class GripperConstants {
  public static final int talon = 40;
  public static final int laser = 41;

  public static final double AP = 1;
  public static final double A23 = -12;
  public static final double AN = 9;

  public static final double current = 60;
  public static final double currentLower = 40;
  public static final double currentLowerTime = 0.5;
  public static final boolean inverted = true;

  public static final double gearing = 3.0;
  public static final double moi = 0.004;

  public static final double proximityThreshold = 0.1;

  public static final ProximityPeriod period = ProximityPeriod.k40ms;
}
