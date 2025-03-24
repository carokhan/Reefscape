package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
  public static final int id = 1;
  public static final int length = 60;

  public static enum Mode {
    OFF(Color.kBlack),
    ALGAE(Color.kTeal),
    CORAL(Color.kPurple),
    CONFIRM_SCORE(Color.kGreen),
    IDLE(Color.kBlue),
    INVALID(Color.kRed),
    DISABLED(Color.kYellow);

    public final Color color;

    private Mode(Color color) {
      this.color = color;
    }
  }
}
