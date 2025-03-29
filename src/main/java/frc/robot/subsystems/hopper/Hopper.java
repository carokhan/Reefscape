package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.proximity.ProximityIO;
import frc.robot.subsystems.proximity.ProximityIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private final ProximityIO proximityIO;
  private final ProximityIOInputsAutoLogged proximityInputs = new ProximityIOInputsAutoLogged();

  public Hopper(HopperIO io, ProximityIO proximityIO) {
    this.io = io;
    this.proximityIO = proximityIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    proximityIO.updateInputs(proximityInputs);

    Logger.processInputs("Hopper", inputs);
    Logger.processInputs("Hopper/Proximity", proximityInputs);
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

  public boolean getDetected() {
    return proximityInputs.detected;
  }

  public double getVoltage() {
    return inputs.motorAppliedVolts;
  }
}
