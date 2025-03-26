package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private final Alert climbDisconnected =
      new Alert("Climb motor disconnected!", Alert.AlertType.kWarning);

  @AutoLogOutput(key = "Climber/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    climbDisconnected.set(!inputs.motorConnected);

    if (DriverStation.isDisabled()) {
      io.setVoltage(0);
    }
  }

  public Command setPosition(Rotation2d position) {
    return this.run(() -> io.setPosition(position));
  }
}
