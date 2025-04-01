package frc.robot.subsystems.proximity;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;

public class ProximityIORedux implements ProximityIO {
  private final Canandcolor canandcolor;
  private CanandcolorSettings settings = new CanandcolorSettings();

  public double maxThreshold;

  public ProximityIORedux(int id) {
    canandcolor = new Canandcolor(id);
    canandcolor.resetFactoryDefaults();
    settings = canandcolor.getSettings();

    settings.setProximityIntegrationPeriod(ProximityPeriod.k5ms);
    settings.setAlignProximityFramesToIntegrationPeriod(true);
    canandcolor.setSettings(settings);
  }

  public ProximityIORedux(int id, double maxThreshold) {
    this(id, maxThreshold, 0);
    this.maxThreshold = maxThreshold;
  }

  public ProximityIORedux(int id, double maxThreshold, double minThreshold) {
    canandcolor = new Canandcolor(id);
    canandcolor.resetFactoryDefaults();
    settings = canandcolor.getSettings();

    settings.setProximityIntegrationPeriod(ProximityPeriod.k5ms);
    settings.setAlignProximityFramesToIntegrationPeriod(true);
    canandcolor.setSettings(settings);
  }

  public void updateInputs(ProximityIOInputsAutoLogged inputs) {
    inputs.connected = canandcolor.isConnected();
    inputs.raw = canandcolor.getProximity();
    inputs.detected = canandcolor.getProximity() < maxThreshold;
    inputs.tempCelsius = canandcolor.getTemperature();
  }

  public void setSimDetected(boolean detected) {}
}
