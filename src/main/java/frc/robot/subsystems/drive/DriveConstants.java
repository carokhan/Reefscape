package frc.robot.subsystems.drive;

public class DriveConstants {
  public static class Characterization {
    public static final double ffDelay = 2.0; // Secs
    public static final double ffRampRate = 0.1; // Volts/Sec
    public static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
    public static final double wheelRadiusRampRate = 0.05; // Rad/Sec^2
  }

  public static class ControlConstants {
    public static final double kPAngle = 5.0;
    public static final double kDAngle = 0.4;
    public static final double angleMaxVelocity = 8.0;
    public static final double angleMaxAccel = 20.0;
  }
}
;
