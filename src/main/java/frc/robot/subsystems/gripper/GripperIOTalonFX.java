package frc.robot.subsystems.gripper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class GripperIOTalonFX implements GripperIO {
    private final TalonFX talon;

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Angle> position;

    private final VoltageOut voltageOut = new VoltageOut(0.0);

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    public GripperIOTalonFX() {
        talon = new TalonFX(GripperConstants.talon, "rio");
        config.MotorOutput.Inverted = GripperConstants.inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = GripperConstants.current;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit = GripperConstants.currentLower;
        config.CurrentLimits.SupplyCurrentLowerTime = GripperConstants.currentLowerTime;
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
    }

    @Override
    public void updateInputs(GripperIOInputsAutoLogged inputs) {
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
