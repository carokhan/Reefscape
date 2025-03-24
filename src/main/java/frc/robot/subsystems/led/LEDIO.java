package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {}

  public default void updateInputs(LEDIOInputs inputs) {}

  public default void set(int i, Color color) {}

  public default void solid(Color color) {
    for (int i = 0; i < LEDConstants.length; i++) {
      set(i, color);
    }
  }
}
