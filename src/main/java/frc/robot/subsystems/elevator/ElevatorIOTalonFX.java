// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Elevator IO using TalonFXs. */
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX talon;
  private final TalonFX followerTalon;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(false);
  private final MotionMagicVoltage positionTorque = new MotionMagicVoltage(0.0);

  // misusing type system here - these correspond to linear meters, NOT rotations
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  private final StatusSignal<Voltage> followerVoltage;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemperature;

  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

  public ElevatorIOTalonFX() {
    talon = new TalonFX(ElevatorConstants.talon, "rio");
    followerTalon = new TalonFX(ElevatorConstants.followerTalon, "rio");
    followerTalon.setControl(new Follower(talon.getDeviceID(), true));

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kG = ElevatorConstants.kG[0];
    config.Slot0.kS = ElevatorConstants.kS[0];
    config.Slot0.kV = ElevatorConstants.kV;
    config.Slot0.kA = ElevatorConstants.kA;
    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kD = ElevatorConstants.kD;

    config.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot1.kG = ElevatorConstants.kG[1];
    config.Slot1.kS = ElevatorConstants.kS[1];
    config.Slot1.kV = ElevatorConstants.kV;
    config.Slot1.kA = ElevatorConstants.kA;
    config.Slot1.kP = ElevatorConstants.kP;
    config.Slot1.kD = ElevatorConstants.kD;

    config.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot2.kG = ElevatorConstants.kG[2];
    config.Slot2.kS = ElevatorConstants.kS[2];
    config.Slot2.kV = ElevatorConstants.kV;
    config.Slot2.kA = ElevatorConstants.kA;
    config.Slot2.kP = ElevatorConstants.kP;
    config.Slot2.kD = ElevatorConstants.kD;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ElevatorConstants.rampPeriod;

    config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.maxAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.maxVelocity;
    config.MotionMagic.MotionMagicExpo_kV = ElevatorConstants.kVExpo;
    config.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.kAExpo;

    config.Feedback.SensorToMechanismRatio =
        ElevatorConstants.gearing / (2 * Math.PI * ElevatorConstants.drumRadius);

    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorCurrent;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyCurrent;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.supplyCurrentLow;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    voltage = talon.getMotorVoltage();
    statorCurrent = talon.getTorqueCurrent();
    supplyCurrent = talon.getSupplyCurrent();
    temperature = talon.getDeviceTemp();
    followerVoltage = followerTalon.getMotorVoltage();
    followerStatorCurrent = followerTalon.getTorqueCurrent();
    followerSupplyCurrent = followerTalon.getSupplyCurrent();
    followerTemperature = followerTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        voltage,
        supplyCurrent,
        statorCurrent,
        temperature,
        followerVoltage,
        followerSupplyCurrent,
        followerStatorCurrent,
        followerTemperature);

    talon.optimizeBusUtilization();
    followerTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(final ElevatorIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        position,
        velocity,
        voltage,
        statorCurrent,
        supplyCurrent,
        temperature,
        followerVoltage,
        followerSupplyCurrent,
        followerStatorCurrent,
        followerTemperature);
    inputs.positionMeters = position.getValueAsDouble();
    inputs.velocityMetersPerSec = velocity.getValueAsDouble();

    inputs.motorAppliedVolts = voltage.getValueAsDouble();
    inputs.motorStatorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.motorSupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.motorTempCelsius = temperature.getValueAsDouble();

    inputs.followerAppliedVolts = followerVoltage.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTempCelsius = followerTemperature.getValueAsDouble();

    inputs.motorConnected =
        motorConnectedDebounce.calculate(
            BaseStatusSignal.isAllGood(voltage, statorCurrent, supplyCurrent, temperature));

    inputs.followerConnected =
        followerConnectedDebounce.calculate(
            BaseStatusSignal.isAllGood(
                followerVoltage,
                followerStatorCurrent,
                followerSupplyCurrent,
                followerTemperature));
  }

  @Override
  public void setTarget(final double meters) {
    talon.setControl(positionTorque.withPosition(meters));
  }

  @Override
  public void setVoltage(final double voltage) {
    talon.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void resetEncoder(final double position) {
    talon.setPosition(position);
  }

  @Override
  public void setPDFF(int slot, double kP, double kD, double kS, double kG) {
    var slotConfigs =
        new SlotConfigs()
            .withKP(kP)
            .withKI(ElevatorConstants.kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(ElevatorConstants.kV)
            .withKA(ElevatorConstants.kA)
            .withKG(kG);
    slotConfigs.SlotNumber = slot;
    talon.getConfigurator().apply(slotConfigs, 0.0);
  }

  @Override
  public void setMagic(double velocity, double acceleration) {
    talon
        .getConfigurator()
        .apply(
            config.MotionMagic.withMotionMagicCruiseVelocity(velocity)
                .withMotionMagicAcceleration(acceleration),
            0.0);
  }

  @Override
  public void setSlot(int slot) {
    positionTorque.withSlot(slot);
  }
}
