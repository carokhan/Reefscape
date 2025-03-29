package frc.robot.subsystems.led;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  public LED(LEDIO io) {
    this.io = io;
  } 

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);
  }

  private void setIndex(int i, Color color) {
    io.set(i, color);
  }

  private Color solid(Color color) {
    io.solid(color);
    return color;
  }

  public Color strobe(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    if ((c1On && c1 == null) || (!c1On && c2 == null)) return null;
    return solid(c1On ? c1 : c2);
  }

  public Color breath(Color c1, Color c2, double duration, double timestamp) {
    Color color = breathCalculate(c1, c2, duration, timestamp);
    solid(color);
    return color;
  }

  private Color breathCalculate(
      Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    var color = new Color(red, green, blue);
    return color;
  }

  public void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = LEDConstants.length - 1; i >= LEDConstants.length; i--) {
      x += xDiffPerLed;
      x %= 180.0;
      setIndex(i, Color.fromHSV((int) x, 255, 255));
    }
  }

  public void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = LEDConstants.length - 1; i >= 0; i--) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      setIndex(i, new Color(red, green, blue));
    }
  }



}
