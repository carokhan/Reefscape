// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static frc.robot.util.SparkUtil.sparkStickyFault;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class HopperIOKraken implements HopperIO {
    TalonFX talon;

    Debouncer currentDebounce = new Debouncer(0.2);

    public HopperIOKraken() {
        talon = new TalonFX(HopperConstants.spark);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = HopperConstants.current;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    @Override
    public void setVoltage(double voltage) {
        talon.setVoltage(voltage);
    }

    @Override
    public void updateInputs(HopperIOInputsAutoLogged inputs) {
        inputs.motorAppliedVolts = talon.getMotorVoltage().getValueAsDouble();
        inputs.motorCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();
        inputs.motorTempCelsius = talon.getDeviceTemp().getValueAsDouble();
        inputs.motorVelocityRPM = talon.getVelocity().getValueAsDouble() * 60.0;
        inputs.motorConnected = talon.isAlive();
        inputs.motorStalled = currentDebounce.calculate((Math.abs(talon.getTorqueCurrent().getValueAsDouble() - 40.0) < 5.0) && ((talon.getVelocity().getValueAsDouble() * 60) < 100.0));
    }
}
