package frc.robot.subsystems.hopper;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;

public class HopperIOSpark implements HopperIO {
  private final SparkMax spark = new SparkMax(HopperConstants.spark, MotorType.kBrushless);
  private final RelativeEncoder encoder = spark.getEncoder();

  private final Debouncer sparkConnectedDebounce = new Debouncer(0.5);
  private final Debouncer currentDebounce = new Debouncer(0.2);

  public HopperIOSpark() {
    var sparkConfig = new SparkMaxConfig();
    sparkConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HopperConstants.current)
        .voltageCompensation(12.0);

    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void processInputs(HopperIOInputsAutoLogged inputs) {
    sparkStickyFault = false;
    ifOk(spark, spark::getAppliedOutput, (value) -> inputs.motorAppliedVolts = value);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.motorCurrentAmps = value);
    ifOk(spark, spark::getMotorTemperature, (value) -> inputs.motorTempCelsius = value);
    ifOk(spark, encoder::getVelocity, (value) -> inputs.motorVelocityRPM = value);
    inputs.motorConnected = sparkConnectedDebounce.calculate(!sparkStickyFault);
    inputs.motorStalled =
        currentDebounce.calculate(
            (Math.abs(spark.getOutputCurrent() - 40.0) < 5) && (encoder.getVelocity() < 100.0));
  }

  @Override
  public void setVoltage(double voltage) {
    spark.setVoltage(voltage);
  }
}
