package frc.robot.subsystems.outtake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  public static class OuttakeIOInputs {
    public boolean motorConnected = false;
    public double motorPositionRots = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
    public double motorTempCelsius = 0.0;
    public double motorVelocityRPM = 0.0;
    public boolean motorStalled = false;
  }

  public default void updateInputs(final OuttakeIOInputsAutoLogged inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setPosition(Rotation2d position) {}

  public default void resetEncoder(double position) {}
}
