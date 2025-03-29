package frc.robot.util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ExtendedDCMotor extends DCMotor {
    public ExtendedDCMotor(double nominalVoltageVolts, double freeSpeedRadPerSec, double freeCurrentAmps, double KvRadPerSecPerVolt, double KtNMPerAmp, int numMotors) {
        super(nominalVoltageVolts, freeSpeedRadPerSec, freeCurrentAmps, KvRadPerSecPerVolt, KtNMPerAmp, numMotors);
    }
    public static DCMotor getKrakenX44(int numMotors) {
        return new DCMotor(
            12.0, 4.05, 275, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(7530), numMotors);
    }
}
