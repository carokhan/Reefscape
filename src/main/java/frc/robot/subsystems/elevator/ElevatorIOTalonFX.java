// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Elevator IO using TalonFXs. */
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX motor = new TalonFX(ElevatorConstants.talon, "rio");
  private final TalonFX follower = new TalonFX(ElevatorConstants.followerTalon, "rio");

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(false);
  private final MotionMagicVoltage positionTorque = new MotionMagicVoltage(0.0);

  // misusing type system here - these correspond to linear meters, NOT rotations
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();
  private final StatusSignal<Temperature> temp = motor.getDeviceTemp();

  public ElevatorIOTalonFX() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Carriage position meters in direction of elevator
    config.Feedback.SensorToMechanismRatio =
        ElevatorConstants.gearing / (2 * Math.PI * ElevatorConstants.drumRadius);

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kG = 0.65;
    config.Slot0.kS = 0.15;
    config.Slot0.kV = 3.34;
    config.Slot0.kA = 0.1;
    config.Slot0.kP = 150.0;
    config.Slot0.kD = 10.0;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 12.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 4.5;

    config.MotionMagic.MotionMagicExpo_kV = 1.9;
    config.MotionMagic.MotionMagicExpo_kA = 0.1141;

    motor.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(motor.getDeviceID(), true));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, voltage, statorCurrent, supplyCurrent, temp);
    motor.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(final ElevatorIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(position, velocity, voltage, statorCurrent, supplyCurrent, temp);
    inputs.positionMeters = position.getValueAsDouble();
    inputs.velocityMetersPerSec = velocity.getValueAsDouble();
    inputs.motorAppliedVolts = voltage.getValueAsDouble();
    inputs.motorStatorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.motorSupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.motorTempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setTarget(final double meters) {
    motor.setControl(positionTorque.withPosition(meters));
  }

  @Override
  public void setVoltage(final double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void resetEncoder(final double position) {
    motor.setPosition(position);
  }
}
