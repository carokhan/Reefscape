package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public boolean motorConnected = false;
    public double motorVelocityRPM = 0.0;
    public double motorPositionRad = 0.0;
    public double motorTempCelsius = 0.0;
    public double motorCurrentAmps = 0.0;
    public double motorAppliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  public default void updateInputs(final ClimberIOInputsAutoLogged inputs) {}

  public default void setVoltage(final double volts) {}

  public default void resetEncoder(final double position) {}
}
