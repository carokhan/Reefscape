package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.LoggedTunableNumber;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  public double currentFilterValue = 0.0;

  public boolean hasZeroed = false;

  private double setpoint = 0.0;

  private final SysIdRoutine voltageSysid;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
  private static final LoggedTunableNumber[] kS = {
      new LoggedTunableNumber("Elevator/kS/Stage1"),
      new LoggedTunableNumber("Elevator/kS/Stage2"),
      new LoggedTunableNumber("Elevator/kS/Stage3")
  };
  private static final LoggedTunableNumber[] kG = {
      new LoggedTunableNumber("Elevator/kG/Stage1"),
      new LoggedTunableNumber("Elevator/kG/Stage2"),
      new LoggedTunableNumber("Elevator/kG/Stage3")
  };
  private static final LoggedTunableNumber[] kA = {
      new LoggedTunableNumber("Elevator/kA/Stage1"),
      new LoggedTunableNumber("Elevator/kA/Stage2"),
      new LoggedTunableNumber("Elevator/kA/Stage3")
  };
  private static final LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec",
      ElevatorConstants.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
      "Elevator/MaxAccelerationMetersPerSec2", ElevatorConstants.maxAcceleration);

  static {
    kP.initDefault(ElevatorConstants.kP);
    kD.initDefault(ElevatorConstants.kD);

    for (int stage = 0; stage < 3; stage++) {
      kS[stage].initDefault(ElevatorConstants.kS[stage]);
      kG[stage].initDefault(ElevatorConstants.kG[stage]);
    }
  }

  public Elevator(ElevatorIO io) {
    this.io = io;
    voltageSysid = new SysIdRoutine(
        new Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Elevator/SysIdTestStateVolts", state.toString())),
        new Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    currentFilterValue = currentFilter.calculate(inputs.motorStatorCurrentAmps);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      for (int stage = 0; stage <= 2; stage++) {
          io.setPDFF(stage, kP.get(), kD.get(), kS[stage].get(), kG[stage].get());
      }
  });
  
  }

  public Command setExtension(DoubleSupplier meters) {
    return this.run(
        () -> {
          io.setTarget(meters.getAsDouble());
          setpoint = meters.getAsDouble();
        });
  }

  public Command setExtension(double meters) {
    return this.setExtension(() -> meters);
  }

  public Command hold() {
    return Commands.sequence(
        setExtension(() -> inputs.positionMeters).until(() -> true), this.run(() -> {
        }));
  }

  public Command runCurrentZeroing() {
    return this.run(
        () -> {
          io.setVoltage(-2.0);
          setpoint = 0.0;
        })
        .until(() -> Math.abs(currentFilterValue) > 50.0)
        .finallyDo(
            (interrupted) -> {
              if (!interrupted) {
                io.resetEncoder(0.0);
                hasZeroed = true;
              }
            });
  }

  public Command runSysid() {
    final Function<SysIdRoutine, Command> runSysid = (routine) -> Commands.sequence(
        routine
            .quasistatic(SysIdRoutine.Direction.kForward)
            .until(() -> inputs.positionMeters > Units.inchesToMeters(50.0)),
        Commands.waitUntil(() -> inputs.velocityMetersPerSec < 0.1),
        routine
            .quasistatic(SysIdRoutine.Direction.kReverse)
            .until(() -> inputs.positionMeters < Units.inchesToMeters(10.0)),
        Commands.waitUntil(() -> Math.abs(inputs.velocityMetersPerSec) < 0.1),
        routine
            .dynamic(SysIdRoutine.Direction.kForward)
            .until(() -> inputs.positionMeters > Units.inchesToMeters(50.0)),
        Commands.waitUntil(() -> inputs.velocityMetersPerSec < 0.1),
        routine
            .dynamic(SysIdRoutine.Direction.kReverse)
            .until(() -> inputs.positionMeters < Units.inchesToMeters(10.0)));
    return Commands.sequence(runCurrentZeroing(), runSysid.apply(voltageSysid));
  }

  public Command setVoltage(double voltage) {
    return this.run(
        () -> {
          io.setVoltage(voltage);
        });
  }

  public Command setVoltage(DoubleSupplier voltage) {
    return this.setVoltage(voltage.getAsDouble());
  }

  public double getExtensionMeters() {
    return inputs.positionMeters;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public boolean isNearExtension(double expected) {
    return MathUtil.isNear(expected, inputs.positionMeters, 0.05);
  }

  public boolean isNearExtension(double expected, double toleranceMeters) {
    return MathUtil.isNear(expected, inputs.positionMeters, toleranceMeters);
  }
}
