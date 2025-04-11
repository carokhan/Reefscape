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

    config.statusLedOffWhenActive = true;
    config.disableWhenLOS = true;
    config.stripType = LEDStripType.RGB;

    config.vBatOutputMode = VBatOutputMode.Off;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);

    candle.setLEDs(255, 0, 0);
  }
}
