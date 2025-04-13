package frc.robot.subsystems.proximity;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;
import edu.wpi.first.math.filter.LinearFilter;

public class ProximityIORedux implements ProximityIO {
  private final Canandcolor canandcolor;
  private CanandcolorSettings settings = new CanandcolorSettings();

  public double maxThreshold;
  private final LinearFilter proximityFilter = LinearFilter.movingAverage(5);

  public ProximityIORedux(int id, ProximityPeriod period) {
    canandcolor = new Canandcolor(id);
    canandcolor.resetFactoryDefaults();
    settings = canandcolor.getSettings();

    settings.setProximityIntegrationPeriod(period);
    settings.setAlignProximityFramesToIntegrationPeriod(true);
    canandcolor.setSettings(settings);
  }

  public ProximityIORedux(int id, double maxThreshold, ProximityPeriod period) {
    this(id, maxThreshold, 0, period);
    this.maxThreshold = maxThreshold;
  }

  public ProximityIORedux(
      int id, double maxThreshold, double minThreshold, ProximityPeriod period) {
    canandcolor = new Canandcolor(id);
    canandcolor.resetFactoryDefaults();
    settings = canandcolor.getSettings();

    settings.setProximityIntegrationPeriod(period);
    settings.setAlignProximityFramesToIntegrationPeriod(true);
    canandcolor.setSettings(settings);
  }

  public void updateInputs(ProximityIOInputsAutoLogged inputs) {
    inputs.connected = canandcolor.isConnected();
    inputs.raw = proximityFilter.calculate(canandcolor.getProximity());
    inputs.detected = canandcolor.getProximity() < maxThreshold;
    inputs.tempCelsius = canandcolor.getTemperature();
  }

  public void setSimDetected(boolean detected) {}
}
