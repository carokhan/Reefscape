package frc.robot.subsystems.proximity;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.DigoutFrameTrigger;
import com.reduxrobotics.sensors.canandcolor.DigoutPinConfig;
import com.reduxrobotics.sensors.canandcolor.HSVDigoutConfig;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;

public class ProximityIORedux implements ProximityIO {
  private final Canandcolor canandcolor;
  private CanandcolorSettings settings = new CanandcolorSettings();

  public ProximityIORedux(int id) {
    canandcolor = new Canandcolor(id);
    canandcolor.resetFactoryDefaults();
    settings = canandcolor.getSettings();
    settings.setProximityFramePeriod(0.05);
    settings.setDigoutFramePeriod(0.5);
    settings.setStatusFramePeriod(0.5);
    settings.setProximityIntegrationPeriod(ProximityPeriod.k5ms);
    settings.setAlignProximityFramesToIntegrationPeriod(false);
    canandcolor.setSettings(settings);
  }

  public ProximityIORedux(int id, double maxThreshold) {
    this(id, maxThreshold, 0);
  }

  public ProximityIORedux(int id, double maxThreshold, double minThreshold) {
    canandcolor = new Canandcolor(id);
    canandcolor.resetFactoryDefaults();
    settings = canandcolor.getSettings();
    settings.setProximityFramePeriod(0.05);
    settings.setDigoutFramePeriod(0.5);
    settings.setStatusFramePeriod(0.5);
    settings.setProximityIntegrationPeriod(ProximityPeriod.k5ms);
    settings.setAlignProximityFramesToIntegrationPeriod(false);
    settings.setDigoutPinConfig(
        canandcolor.digout1().channelIndex(), DigoutPinConfig.kDigoutLogicActiveHigh);
    settings.setDigoutFrameTrigger(
        canandcolor.digout1().channelIndex(), DigoutFrameTrigger.kRisingAndFalling);
    canandcolor.setSettings(settings);
    canandcolor
        .digout1()
        .configureSlots(
            new HSVDigoutConfig().setMaxProximity(maxThreshold).setMinProximity(minThreshold));
  }

  public void updateInputs(ProximityIOInputsAutoLogged inputs) {
    inputs.connected = canandcolor.isConnected();
    inputs.raw = canandcolor.getProximity();
    inputs.detected = canandcolor.digout1().getValue();
    inputs.tempCelsius = canandcolor.getTemperature();
  }

  public void setSimDetected(boolean detected) {}
}
