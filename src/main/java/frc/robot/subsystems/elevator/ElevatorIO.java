package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double targetPositionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;

    public double motorAppliedVolts = 0.0;
    public double motorStatorCurrentAmps = 0.0;
    public double motorSupplyCurrentAmps = 0.0;
    public double motorVelocityMetersPerSecond = 0.0;

    public double followerAppliedVolts = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerVelocityMetersPerSecond = 0.0;

    public boolean motorConnected = false;
    public double motorTempCelsius = 0.0;

    public boolean followerConnected = false;
    public double followerTempCelsius = 0.0;
  }

  public default void updateInputs(final ElevatorIOInputsAutoLogged inputs) {}

  public default void setTarget(final double meters) {}

  public default void setVoltage(final double voltage) {}

  public default void stop() {
    setVoltage(0);
  }

  public default void resetEncoder(final double position) {}

  public default void resetEncoder() {
    resetEncoder(0.0);
  }

  public default void setPDFF(
      int slot, double kP, double kD, double kS, double kG) {}
}
