// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** WPILib physics model based elevator sim. */
public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim physicsSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(2),
          ElevatorConstants.gearing,
          Units.lbsToKilograms(15),
          ElevatorConstants.drumRadius,
          0.0,
          ElevatorConstants.AN,
          true,
          0.0);
  private double volts = 0.0;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(40.0, 0.0, 0.1, new Constraints(5.0, 10.0));
  private final ElevatorFeedforward ff =
      new ElevatorFeedforward(
          0.0,
          0.06,
          (DCMotor.getKrakenX60(1).KvRadPerSecPerVolt * ElevatorConstants.drumRadius)
              / ElevatorConstants.gearing);

  @Override
  public void updateInputs(final ElevatorIOInputsAutoLogged inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }
    physicsSim.update(0.020);
    inputs.positionMeters = physicsSim.getPositionMeters();
    inputs.velocityMetersPerSec = physicsSim.getVelocityMetersPerSecond();
    inputs.motorAppliedVolts = volts;
    inputs.motorStatorCurrentAmps = physicsSim.getCurrentDrawAmps();
    inputs.motorTempCelsius = 20.0;
  }

  @Override
  public void setTarget(final double meters) {
    setVoltage(
        pid.calculate(physicsSim.getPositionMeters(), meters)
            + ff.calculate(pid.getSetpoint().velocity));
  }


  @Override
  public void setVoltage(final double voltage) {
    volts = voltage;
    physicsSim.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  @Override
  public void resetEncoder(final double position) {
    // sim always has a perfectly accurate encoder
  }
}