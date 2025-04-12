package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.robot.subsystems.led.LEDConstants.Mode;

public class LEDIOCandle implements LEDIO {
  private final CANdle candle;
  private final CANdleConfiguration config = new CANdleConfiguration();
  private Mode mode = Mode.OFF;

  public LEDIOCandle() {
    candle = new CANdle(LEDConstants.candleId, "rio");

    config.stripType = LEDStripType.GRB;
    config.brightnessScalar = 0.5;
    config.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(config);

    candle.setLEDs(0, 0, 255, 0, 0, LEDConstants.length);
  }
}
