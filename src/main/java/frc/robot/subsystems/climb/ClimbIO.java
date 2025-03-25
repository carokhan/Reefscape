package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public class ClimbIOInputs {
    public boolean motorConnected = false;
    public double motorPositionRad = 0.0;
    public double motorVelocityRadPerSec = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
    public double motorTempCelsius = 0.0;
  }

  public default void updateInputs(final ClimbIOInputsAutoLogged inputs) {}

  public default void setVoltage(final double volts) {}

  public default void stop() {}

  public default void resetEncoder(final double position) {}

  public default void setBrakeMode(boolean enabled) {}
}
