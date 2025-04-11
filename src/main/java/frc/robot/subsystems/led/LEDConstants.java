package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
  public static final int rioId = 0;
  public static final int candleId = 60;

  public static final int length = 90;

  public static enum Mode {
    OFF(Color.kBlack),
    INTAKING(Color.kOrange),
    ALGAE_READY(Color.kTeal),
    CORAL_READY(Color.kPurple),
    CONFIRM(Color.kGreen),
    INVALID(Color.kRed),
    DISABLED(Color.kYellow);

    public final Color color;

    private Mode(Color color) {
      this.color = color;
    }
  }

  public static final double waveExponent = 0.4;
}
