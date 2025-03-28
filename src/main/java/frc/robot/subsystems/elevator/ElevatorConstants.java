package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final int talon = 10;
  public static final int followerTalon = 11;

  public static final double intake = 0;
  public static final double AP = 0.05;
  public static final double L1 = 0.33;
  public static final double A2 = 0.48;
  public static final double L2 = 0.63;
  public static final double A3 = 0.81;
  public static final double L3 = 1.05;
  public static final double L4 = 1.75;
  public static final double AN = 1.76;

  public static final double travel = Units.inchesToMeters(69.0);
  public static final double mass = Units.lbsToKilograms(25.1);

  public static final double stageOneTravel = Units.inchesToMeters(24);
  public static final double stageTwoTravel = Units.inchesToMeters(24);
  public static final double carriageTravel = Units.inchesToMeters(21);

  public static final double drumRadius = 5.0 / 1000.0 * 36 / (2.0 * Math.PI);
  public static final double gearing = (9.0 / 1.0);
  public static final double positionConversionFactor = drumRadius * 2 * Math.PI / gearing;
  public static final double tolerance = .005;

  public static final int statorCurrent = 60;
  public static final int supplyCurrent = 70;
  public static final int supplyCurrentLow = 40;

  public static final double[] kS = {0.4, 0.4, 0.6};
  public static final double[] kG = {0.54, 0.43, 0.32};
  public static final double kV = 0.584;
  public static final double kVExpo = 1.9;
  public static final double kA = 0.0;
  public static final double kAExpo = 0.1189;

  public static final double kP = 5.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double rampPeriod = 0.1;

  public static final double maxVelocity = // 0.3;
      0.3; // 3.5
  public static final double maxAcceleration = // 0.3;
      0.3; // 70.2

  public static final double visualizerOffset = 0.0;
}
