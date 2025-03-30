package frc.robot.subsystems.proximity;

import java.util.function.DoubleSupplier;

public class ProximityIOSim implements ProximityIO {
  private boolean detected = false;
  private double threshold = 1;
  private DoubleSupplier distance = null;

  public ProximityIOSim(double threshold) {
    this.threshold = threshold;
  }

  public ProximityIOSim(double threshold, DoubleSupplier distance) {
    this.threshold = threshold;
  }

  public void updateInputs(ProximityIOInputsAutoLogged inputs) {
    inputs.connected = true;
    if (distance == null) {
      inputs.raw = (detected ? threshold : 0.0);
      inputs.detected = detected;
    } else {
      inputs.raw = distance.getAsDouble();
      inputs.detected = distance.getAsDouble() < threshold;
    }

    inputs.tempCelsius = 20.0;
  }

  @Override
  public void setSimDetected(boolean detected) {
    this.detected = detected;
  }
}
