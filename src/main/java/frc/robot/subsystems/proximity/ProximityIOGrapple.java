package frc.robot.subsystems.proximity;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class ProximityIOGrapple implements ProximityIO {
  private final LaserCan lasercan;

  private final double threshold;

  public ProximityIOGrapple(
      int id, RangingMode rangingMode, RegionOfInterest regionOfInterest, double threshold) {
    lasercan = new LaserCan(id);

    try {
      lasercan.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
      lasercan.setRangingMode(rangingMode);
      lasercan.setRegionOfInterest(regionOfInterest);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }

    this.threshold = threshold;
  }

  public ProximityIOGrapple(int id, RangingMode rangingMode, RegionOfInterest regionOfInterest) {
    this(id, rangingMode, regionOfInterest, 4000);
  }

  public void updateInputs(ProximityIOInputsAutoLogged inputs) {
    var measurement = lasercan.getMeasurement();
    inputs.connected = (measurement != null);
    inputs.raw = measurement.distance_mm;
    inputs.detected =
        (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
            && (measurement.distance_mm < threshold);
    inputs.tempCelsius = 20.0;
  }
}
