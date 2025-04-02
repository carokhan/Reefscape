package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  public double currentFilterValue = 0.0;

  public boolean hasZeroed = false;

  private double setpoint = 0.0;

  private final SysIdRoutine voltageSysid;

  @AutoLogOutput private boolean homed = false;

  private Debouncer homingDebouncer = new Debouncer(0.2);

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

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec", ElevatorConstants.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
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
    voltageSysid =
        new SysIdRoutine(
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

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          for (int stage = 0; stage <= 2; stage++) {
            io.setPDFF(stage, kP.get(), kD.get(), kS[stage].get(), kG[stage].get());
          }
          ;
          io.setMagic(maxVelocity.get(), maxAcceleration.get());
        });

    io.setSlot((int) (inputs.positionMeters / Units.inchesToMeters(23)));
    io.setTarget(inputs.targetPositionMeters);
  }

  public Command setExtension(DoubleSupplier meters) {
    return this.run(
        () -> {
          Logger.recordOutput("AutoAlign/ElevState", meters.getAsDouble());
          inputs.targetPositionMeters = meters.getAsDouble();
          io.setTarget(meters.getAsDouble());
        });
  }

  public Command setExtension(double meters) {
    return this.setExtension(() -> meters);
  }

  public Command hold() {
    return Commands.sequence(
        setExtension(() -> inputs.positionMeters).until(() -> true), this.run(() -> {}));
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
    final Function<SysIdRoutine, Command> runSysid =
        (routine) ->
            Commands.sequence(
                routine
                    .quasistatic(SysIdRoutine.Direction.kForward)
                    .until(() -> inputs.positionMeters > Units.inchesToMeters(60.0)),
                Commands.waitUntil(() -> inputs.velocityMetersPerSec < 0.1),
                routine
                    .quasistatic(SysIdRoutine.Direction.kReverse)
                    .until(() -> inputs.positionMeters < Units.inchesToMeters(10.0)),
                Commands.waitUntil(() -> Math.abs(inputs.velocityMetersPerSec) < 0.1),
                routine
                    .dynamic(SysIdRoutine.Direction.kForward)
                    .until(() -> inputs.positionMeters > Units.inchesToMeters(60.0)),
                Commands.waitUntil(() -> inputs.velocityMetersPerSec < 0.1),
                routine
                    .dynamic(SysIdRoutine.Direction.kReverse)
                    .until(() -> inputs.positionMeters < Units.inchesToMeters(10.0)));
    return Commands.sequence(runCurrentZeroing(), runSysid.apply(voltageSysid));
  }

  public Command reset() {
    return this.run(
        () -> {
          inputs.targetPositionMeters = 0.0;
        });
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

  public double getTargetExtensionMeters() {
    return inputs.targetPositionMeters;
  }

  public boolean isNearExtension() {
    return MathUtil.isNear(inputs.targetPositionMeters, inputs.positionMeters, 0.1);
  }

  public boolean isNearExtension(double expected) {
    return MathUtil.isNear(expected, inputs.positionMeters, 0.1);
  }

  public boolean isNearExtension(double expected, double toleranceMeters) {
    return MathUtil.isNear(expected, inputs.positionMeters, toleranceMeters);
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              homed = false;
              homingDebouncer = new Debouncer(0.2);
              homingDebouncer.calculate(false);
            },
            () -> {
              io.setVoltage(-6);
              homed = homingDebouncer.calculate(Math.abs(inputs.velocityMetersPerSec) <= 0.2);
            })
        .until(() -> homed)
        .andThen(
            () -> {
              io.resetEncoder();
              homed = true;
            });
  }

  public Command staticCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              io.setVoltage(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> inputs.velocityMetersPerSec >= 0.2)
        .finallyDo(
            () -> {
              timer.stop();
              Logger.recordOutput("Elevator/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
