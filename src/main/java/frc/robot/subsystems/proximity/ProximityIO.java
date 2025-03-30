package frc.robot.subsystems.proximity;

import org.littletonrobotics.junction.AutoLog;

public interface ProximityIO {
  @AutoLog
  public static class ProximityIOInputs {
    public boolean connected = false;
    public double raw = 0.0;
    public boolean detected = false;
    public double tempCelsius = 20.0;
  }

  public default void updateInputs(final ProximityIOInputsAutoLogged inputs) {}

  public default void setSimDetected(boolean detected) {}
}
