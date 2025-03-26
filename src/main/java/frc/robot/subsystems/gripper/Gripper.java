package frc.robot.subsystems.gripper;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private final GripperIO io;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  public Gripper(GripperIO io) {
    this.io = io;
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
}
