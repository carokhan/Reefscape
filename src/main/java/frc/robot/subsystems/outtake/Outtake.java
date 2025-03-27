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

  private final ProximityIO proximityIO;
  private final ProximityIOInputsAutoLogged proximityInputs = new ProximityIOInputsAutoLogged();

  public Outtake(OuttakeIO io, ProximityIO proximityIO) {
    this.io = io;
    this.proximityIO = proximityIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    proximityIO.updateInputs(proximityInputs);

    Logger.processInputs("Outtake", inputs);
    Logger.processInputs("Outtake/Proximity", proximityInputs);
  }

  public Command index() {
    return Commands.sequence(
        setVoltage(2.0)
            .until(() -> proximityInputs.detected)
            .unless(() -> proximityInputs.detected),
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
        });
  }

  public Command setVoltage(double voltage) {
    return this.setVoltage(() -> voltage);
  }

  public boolean getDetected() {
    return proximityInputs.detected;
  }
}
