package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
  public static final int rioId = 0;
  public static final int candleId = 60;

  public static final double brightness = 0.5;
  public static final int length = 308;

  public static final Animation coralIntakeAnimation =
      new StrobeAnimation(
          (int) (Color.kWhite.red * 255),
          (int) (Color.kWhite.green * 255),
          (int) (Color.kWhite.blue * 255),
          0,
          0.5,
          length);
  public static final Animation algaeIntakeAnimation =
      new StrobeAnimation(
          (int) Color.kTeal.red, (int) Color.kTeal.green, (int) Color.kTeal.blue, 255, 0.5, length);
  public static final Animation climbReadyAnimation =
      new StrobeAnimation(
          (int) Color.kYellow.red,
          (int) Color.kYellow.green,
          (int) Color.kYellow.blue,
          0,
          0.5,
          length);
  public static final Animation outtakeAnimation =
      new StrobeAnimation(
          (int) Color.kGreen.red,
          (int) Color.kGreen.green,
          (int) Color.kGreen.blue,
          0,
          0.5,
          length);
  public static final Animation revFunnelAnimation =
      new StrobeAnimation(
          (int) Color.kOrange.red,
          (int) Color.kOrange.green,
          (int) Color.kOrange.blue,
          0,
          0.5,
          length);

  public static final Animation endAnimation = new RainbowAnimation();
  public static final Animation idleAnimation = new SingleFadeAnimation(0, 0, 255, 0, 0.85, length);
  public static final Animation coralReadyAnimation =
      new SingleFadeAnimation(255, 255, 255, 0, 0, LEDConstants.length);
  public static final Animation algaeReadyAnimation =
      new SingleFadeAnimation(0, 255, 255, 0, 0, LEDConstants.length);
  public static final Animation climbedAnimation =
      new SingleFadeAnimation(255, 255, 0, 0, 0, LEDConstants.length);

  public static final double waveExponent = 0.4;
}
