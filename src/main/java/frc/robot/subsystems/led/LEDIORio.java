// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIORio implements LEDIO {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LEDIORio() {
    led = new AddressableLED(LEDConstants.rioId);
    buffer = new AddressableLEDBuffer(LEDConstants.length);
    led.setLength(buffer.getLength());
    led.start();
  }

  /** Write data from buffer to leds */
  @Override
  public void updateInputs(LEDIOInputs inputs) {
    led.setData(buffer);
  }

  @Override
  public void set(int i, Color color) {
    buffer.setLED(i, color);
  }
}
