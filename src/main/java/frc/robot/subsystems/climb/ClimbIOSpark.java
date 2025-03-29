package frc.robot.subsystems.climb;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ClimbIOSpark implements ClimbIO {
  private final SparkMax spark;
  private final RelativeEncoder encoder;
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SoftLimitConfig limits = new SoftLimitConfig();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Climb/kP", ClimbConstants.kPSpark);

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private double targetPositionRad;

  public ClimbIOSpark() {
    spark = new SparkMax(ClimbConstants.spark, MotorType.kBrushless);
    encoder = spark.getEncoder();

    limits
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(ClimbConstants.climbed.getRadians())
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(ClimbConstants.stow.getRadians());

    config
        .inverted(ClimbConstants.inverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimbConstants.sparkCurrent)
        .apply(limits);

    config
        .encoder
        .positionConversionFactor((1 / ClimbConstants.gearing) * 2 * Math.PI)
        .velocityConversionFactor((1 / ClimbConstants.gearing) * 2 * Math.PI / 60.0);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(kP.get(), 0.0, 0.0, 0.0);

    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(spark, 5, () -> encoder.setPosition(ClimbConstants.stow.getRadians()));
  }

  @Override
  public void updateInputs(ClimbIOInputsAutoLogged inputs) {
    sparkStickyFault = false;
    ifOk(spark, encoder::getPosition, (value) -> inputs.motorPositionRad = value);
    inputs.targetPositionRad = this.targetPositionRad;
    ifOk(spark, encoder::getVelocity, (value) -> inputs.motorVelocityRadPerSec = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.motorAppliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.motorSupplyCurrentAmps = value);
    ifOk(spark, spark::getMotorTemperature, (value) -> inputs.motorTempCelsius = value);
    inputs.motorConnected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setPosition(Rotation2d position) {
    this.targetPositionRad = position.getRadians();
    spark.getClosedLoopController().setReference(position.getRadians(), ControlType.kPosition);
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void stop() {
    spark.setVoltage(0.0);
  }

  @Override
  public void resetEncoder() {
    tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void resetEncoder(final double position) {
    tryUntilOk(spark, 5, () -> encoder.setPosition(position));
  }
}
