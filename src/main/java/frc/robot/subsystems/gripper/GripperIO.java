package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
  @AutoLog
  public static class GripperIOInputs {
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

  public default void processInputs(final GripperIOInputsAutoLogged inputs) {}

  public default void setVoltage(double voltage) {}
}
