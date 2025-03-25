package frc.robot.subsystems.climb;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private final Debouncer climbConnectedDebounce = new Debouncer(0.5);
  private final Alert climbDisconnected =
      new Alert("Climb motor disconnected!", Alert.AlertType.kWarning);

  public Climb(ClimbIO io) {
    this.io = io;
    climbIO.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    climbDisconnected.set(!inputs.motorConnected);


  }
}
