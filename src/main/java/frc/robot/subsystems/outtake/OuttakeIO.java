package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  public static class OuttakeIOInputs {
    public boolean motorConnected = false;
    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
    public double motorTempCelsius = 0.0;
    public double motorVelocityRPM = 0.0;
    public boolean motorStalled = false;

    public boolean sensorConnected = false;
    public double sensorRaw = 0.0;
    public boolean sensorDetected = false;
    public double sensorTempCelsius = 20.0;
  }

  public default void processInputs(final OuttakeIOInputsAutoLogged inputs) {}

  public default void setVoltage(double voltage) {}
}
