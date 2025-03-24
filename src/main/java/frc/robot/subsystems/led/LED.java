package frc.robot.subsystems.led;

public class LED {
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  public LED(LEDIO io) {
    this.io = io;
  }
}
