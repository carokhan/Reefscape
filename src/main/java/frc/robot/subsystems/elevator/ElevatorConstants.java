package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Superstructure.CoralTarget;
import java.util.Map;

public class ElevatorConstants {
  public static final int talon = 10;
  public static final int followerTalon = 11;

  public static final double intake = 0.014;
  public static final double AP = 0.05;
  public static final double L1 = 0.33;
  public static final double A2 = 0.36;
  public static final double L2 = 0.66;
  public static final double A3 = 0.81;
  public static final double L3 = 1.1;
  public static final double L4 = 1.76;
  public static final double AN = 1.78;

  public static final Map<CoralTarget, Double> targetToCoral =
      Map.of(
          CoralTarget.L1, L1,
          CoralTarget.L2, L2,
          CoralTarget.L3, L3,
          CoralTarget.L4, L4);

  public static final double confirmTimeout = 0.5;

  public static final double reefRaiseDistance = 1.5;
  public static final double processRaiseDistance = 0.5;
  public static final double netRaiseDistanceLower = 0.75;
  public static final double netRaiseDistanceUpper = 1;

  public static final double travel = Units.inchesToMeters(69.0);
  public static final double mass = Units.lbsToKilograms(25.1);

  public static final double stageOneTravel = Units.inchesToMeters(24);
  public static final double stageTwoTravel = Units.inchesToMeters(24);
  public static final double carriageTravel = Units.inchesToMeters(21);

  public static final double drumRadius = 5.0 / 1000.0 * 36 / (2.0 * Math.PI);
  public static final double gearing = (5.0 / 1.0);
  public static final double positionConversionFactor = drumRadius * 2 * Math.PI / gearing;
  public static final double tolerance = .005;

  public static final int statorCurrent = 120;
  public static final int supplyCurrent = 80;
  public static final int supplyCurrentLow = 40;

  public static final double[] kS = {0.74, 0.74, 0.74}; // 0.2
  public static final double[] kG = {0.459, 0.459, 0.459};
  public static final double kV = 3.122;
  public static final double kVExpo = 1.9;
  public static final double kA = 0.0;
  public static final double kAExpo = 0.1189;

  public static final double kP = 55.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double rampPeriod = 0.1;

  public static final double maxVelocity = // 0.3;
      3.5; // 3.5
  public static final double maxAcceleration = // 0.3;
      10.0; // 70.2

  public static final double visualizerOffset = Units.inchesToMeters(8.5);
}
