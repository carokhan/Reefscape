package frc.robot.subsystems.gripper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.proximity.ProximityIO;
import frc.robot.subsystems.proximity.ProximityIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
  private final GripperIO io;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  private final ProximityIO proximityIO;
  private final ProximityIOInputsAutoLogged proximityInputs = new ProximityIOInputsAutoLogged();

  private final Debouncer currentDebouncer = new Debouncer(0.25);
  private final Debouncer dualDebouncer = new Debouncer(0.25);

  public Gripper(GripperIO io, ProximityIO proximityIO) {
    this.io = io;
    this.proximityIO = proximityIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    proximityIO.updateInputs(proximityInputs);

    Logger.processInputs("Gripper", inputs);
    Logger.processInputs("Gripper/Proximity", proximityInputs);
  }

  public Command setVoltage(DoubleSupplier volts) {
    return this.run(
        () -> {
          io.setVoltage(volts.getAsDouble());
        });
  }

  public Command setVoltage(double voltage) {
    return this.setVoltage(() -> voltage);
  }

  public double getVoltage() {
    return inputs.motorAppliedVolts;
  }

  public boolean getDetected() {
    return proximityInputs.detected;
  }

  public boolean getCurrentDetected() {
    return currentDebouncer.calculate(
        (inputs.motorCurrentAmps > 15) && (Math.abs(inputs.motorVelocityRPM) < 5));
  }

  public boolean getDualDetected() {
    boolean debounced = dualDebouncer.calculate(getCurrentDetected() || getDetected());
    Logger.recordOutput("Gripper/Detected", debounced);
    return debounced;
  }

  public Command setSimDetected(boolean detected) {
    return this.runOnce(
        () -> {
          proximityIO.setSimDetected(detected);
        });
  }
}
