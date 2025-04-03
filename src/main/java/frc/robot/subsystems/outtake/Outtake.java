package frc.robot.subsystems.outtake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.proximity.ProximityIO;
import frc.robot.subsystems.proximity.ProximityIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

  private final ProximityIO coralProximityIO;
  private final ProximityIOInputsAutoLogged coralProximityInputs =
      new ProximityIOInputsAutoLogged();

  private final ProximityIO reefProximityIO;
  private final ProximityIOInputsAutoLogged reefProximityInputs = new ProximityIOInputsAutoLogged();

  public Outtake(OuttakeIO io, ProximityIO coralProximityIO, ProximityIO reefProximityIO) {
    this.io = io;
    this.coralProximityIO = coralProximityIO;
    this.reefProximityIO = reefProximityIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    coralProximityIO.updateInputs(coralProximityInputs);
    reefProximityIO.updateInputs(reefProximityInputs);

    Logger.processInputs("Outtake", inputs);
    Logger.processInputs("Outtake/CoralProximity", coralProximityInputs);
    Logger.processInputs("Outtake/ReefProximity", reefProximityInputs);
  }

  public Command index() {
    return Commands.sequence(
        setVoltage(OuttakeConstants.index)
            .until(() -> coralProximityInputs.detected)
            .unless(() -> coralProximityInputs.detected),
        setVoltage(0));
  }

  public Command jog(double rotations) {
    return Commands.sequence(this.run(() -> io.setPosition(Rotation2d.fromRotations(rotations))));
  }

  public Command hold() {
    return this.jog(inputs.motorPositionRots).until(() -> true).andThen(this.run(() -> {}));
  }

  public Command setVelocity(DoubleSupplier vel) {
    return this.run(() -> io.setVelocity(vel.getAsDouble()));
  }

  public Command setVelocity(double vel) {
    return this.setVelocity(() -> vel);
  }

  public Command setVoltage(DoubleSupplier volts) {
    return this.run(
        () -> {
          io.setVoltage(volts.getAsDouble());
          if (coralProximityInputs.detected) setSimDetected(false);
        });
  }

  public Command setVoltage(double voltage) {
    return this.setVoltage(() -> voltage);
  }

  public boolean getConnected() {
    return coralProximityInputs.connected;
  }

  public boolean getDetected() {
    return coralProximityInputs.detected;
  }

  public double getVoltage() {
    return inputs.motorAppliedVolts;
  }

  public Command setSimDetected(boolean detected) {
    return this.runOnce(
        () -> {
          coralProximityIO.setSimDetected(detected);
        });
  }
}
