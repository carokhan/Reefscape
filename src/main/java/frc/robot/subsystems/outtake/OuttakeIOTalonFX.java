package frc.robot.subsystems.outtake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.ColorPeriod;
import com.reduxrobotics.sensors.canandcolor.DigoutFrameTrigger;
import com.reduxrobotics.sensors.canandcolor.DigoutPinConfig;
import com.reduxrobotics.sensors.canandcolor.HSVDigoutConfig;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.gripper.GripperIOInputsAutoLogged;

public class OuttakeIOTalonFX implements OuttakeIO {
    private final TalonFX talon;
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Angle> position;

    private final VoltageOut voltageOut = new VoltageOut(0.0);

    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    private final Canandcolor canandcolor;
    private CanandcolorSettings settings = new CanandcolorSettings();

    public OuttakeIOTalonFX() {
        talon = new TalonFX(OuttakeConstants.talon, "rio");
        config.MotorOutput.Inverted = OuttakeConstants.inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = OuttakeConstants.current;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> talon.getConfigurator().apply(config));

        position = talon.getPosition();
        velocity = talon.getVelocity();
        voltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        statorCurrent = talon.getTorqueCurrent();
        temperature = talon.getDeviceTemp();

        tryUntilOk(
                5,
                () -> BaseStatusSignal.setUpdateFrequencyForAll(
                        50.0,
                        position,
                        velocity,
                        voltage,
                        supplyCurrent,
                        statorCurrent,
                        temperature));
        tryUntilOk(5, () -> talon.optimizeBusUtilization(0, 1.0));

        canandcolor = new Canandcolor(OuttakeConstants.canandcolor);
        canandcolor.resetFactoryDefaults();
        settings = canandcolor.getSettings();
        settings.setProximityFramePeriod(0.5);
        settings.setDigoutFramePeriod(0.5);
        settings.setStatusFramePeriod(0.5);
        settings.setProximityIntegrationPeriod(ProximityPeriod.k5ms);
        settings.setAlignProximityFramesToIntegrationPeriod(false);
        settings.setDigoutPinConfig(canandcolor.digout1().channelIndex(), DigoutPinConfig.kDigoutLogicActiveHigh);
        settings.setDigoutFrameTrigger(canandcolor.digout1().channelIndex(), DigoutFrameTrigger.kRisingAndFalling);
        canandcolor.setSettings(settings);
        canandcolor.digout1().configureSlots(new HSVDigoutConfig()
                .setMaxProximity(0.5)
                .setMinProximity(0));
    }

    @Override
    public void updateInputs(OuttakeIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(position, velocity, voltage, statorCurrent, supplyCurrent, temperature);
        inputs.motorAppliedVolts = voltage.getValueAsDouble();
        inputs.motorCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.motorTempCelsius = temperature.getValueAsDouble();
        inputs.motorVelocityRPM = velocity.getValueAsDouble();
        inputs.motorStalled = (inputs.motorCurrentAmps > 30) && (inputs.motorVelocityRPM < 100);

        inputs.motorConnected = connectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(voltage, statorCurrent, supplyCurrent, temperature));
    }

    @Override
    public void setVoltage(double voltage) {
        talon.setControl(voltageOut.withOutput(voltage));
    }
}
