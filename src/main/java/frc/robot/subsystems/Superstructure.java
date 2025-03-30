package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Superstructure {
  public static enum CoralTarget {
    IDLE(0, 0),
    L1(ElevatorConstants.L1, OuttakeConstants.L1),
    L2(ElevatorConstants.L2, OuttakeConstants.L234),
    L3(ElevatorConstants.L3, OuttakeConstants.L234),
    L4(ElevatorConstants.L4, OuttakeConstants.L234);

    public final double elevatorHeight;
    public final double outtakeSpeed;

    private CoralTarget(double elevatorHeight, double outtakeSpeed) {
      this.elevatorHeight = elevatorHeight;
      this.outtakeSpeed = outtakeSpeed;
    }
  }

  public static enum AlgaeTarget {
    IDLE(0, 0),
    AP(ElevatorConstants.AP, GripperConstants.AP),
    A2(ElevatorConstants.A2, GripperConstants.A23),
    A3(ElevatorConstants.A3, GripperConstants.A23),
    AN(ElevatorConstants.AN, GripperConstants.AN);

    public final double elevatorHeight;
    public final double gripperSpeed;

    private AlgaeTarget(double elevatorHeight, double gripperSpeed) {
      this.elevatorHeight = elevatorHeight;
      this.gripperSpeed = gripperSpeed;
    }
  }

  public enum State {
    CORAL_PREINTAKE,
    CORAL_TRANSFER,
    CORAL_READY,
    CORAL_OUTTAKE,
    CORAL_PRESCORE,
    CORAL_CONFIRM_L1,
    CORAL_CONFIRM_L2,
    CORAL_CONFIRM_L3,
    CORAL_CONFIRM_L4,
    CORAL_SCORE_L1,
    CORAL_SCORE_L2,
    CORAL_SCORE_L3,
    CORAL_SCORE_L4,
    ALGAE_INTAKE_A2,
    ALGAE_INTAKE_A3,
    ALGAE_READY,
    ALGAE_OUTTAKE,
    ALGAE_PRESCORE_AP,
    ALGAE_PRESCORE_AN,
    ALGAE_CONFIRM_AP,
    ALGAE_CONFIRM_AN,
    ALGAE_SCORE_AP,
    ALGAE_SCORE_AN,
    CLIMB_PREPULL,
    CLIMB_PULL,
    IDLE
  }

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisSpeeds> velocity;

  @AutoLogOutput(key = "Superstructure/Score")
  private final Trigger scoreRequest;

  @AutoLogOutput(key = "Superstructure/Coral Intake")
  private final Trigger coralIntakeRequest;

  @AutoLogOutput(key = "Superstructure/Algae Intake")
  private final Trigger algaeIntakeRequest;

  @AutoLogOutput(key = "Superstructure/Pre Climb")
  private final Trigger preClimbRequest;

  @AutoLogOutput(key = "Superstructure/Climb")
  private final Trigger climbRequest;

  @AutoLogOutput(key = "Superstructure/Cancel Climb")
  private final Trigger cancelClimbRequest;

  @AutoLogOutput(key = "Superstructure/Homing Request")
  private final Trigger homeRequest;

  @AutoLogOutput(key = "Superstructure/Coast Request")
  private final Trigger superstructureCoastRequest;

  @AutoLogOutput(key = "Superstructure/State")
  private State state = State.IDLE;

  private State prevState = State.IDLE;

  private Map<State, Trigger> stateTriggers = new HashMap<State, Trigger>();

  private Timer stateTimer = new Timer();

  private final Hopper hopper;
  private final Elevator elevator;
  private final Outtake outtake;
  private final Gripper gripper;
  private final Climb climb;
  private final LED led;

  private final Supplier<CoralTarget> coralTarget;
  private final Supplier<AlgaeTarget> algaeTarget;

  private final DoubleSupplier driverX;
  private final DoubleSupplier driverY;

  @AutoLogOutput(key = "Superstructure/Visualizer")
  private final LoggedMechanism2d mech2d =
      new LoggedMechanism2d(Units.inchesToMeters(48), Units.inchesToMeters(96));

  private final LoggedMechanismRoot2d mech2dRoot =
      mech2d.getRoot(
          "drive_base", Units.inchesToMeters(24 - 28.5 / 2), Units.inchesToMeters(1.125));
  private final LoggedMechanismLigament2d driveLigament =
      mech2dRoot.append(
          new LoggedMechanismLigament2d(
              "drive", Units.inchesToMeters(28.5), 0, 2, new Color8Bit(Color.kWhite)));

  private final LoggedMechanismRoot2d elevatorRoot =
      mech2d.getRoot("elevator_base", Units.inchesToMeters(24 - 3), Units.inchesToMeters(4.087));
  private final LoggedMechanismLigament2d elevatorLigament =
      elevatorRoot.append(
          new LoggedMechanismLigament2d(
              "elevator", Units.inchesToMeters(0 + 8.5), 92.5, 4, new Color8Bit(Color.kYellow)));
  private final LoggedMechanismLigament2d elevatorTargetLigament =
      elevatorRoot.append(
          new LoggedMechanismLigament2d(
              "elevator_target",
              Units.inchesToMeters(36),
              92.5,
              2,
              new Color8Bit(Color.kPaleGoldenrod)));

  private final LoggedMechanismLigament2d outtakeLigament =
      elevatorLigament.append(
          new LoggedMechanismLigament2d(
              "outtake", Units.inchesToMeters(8), 90, 4, new Color8Bit(Color.kGray)));
  private final LoggedMechanismLigament2d gripperLigament =
      elevatorLigament.append(
          new LoggedMechanismLigament2d(
              "gripper", Units.inchesToMeters(20), 25, 4, new Color8Bit(Color.kGray)));

  private final LoggedMechanismRoot2d hopperRoot =
      mech2d.getRoot("hopper_base", Units.inchesToMeters(24 + 2), Units.inchesToMeters(11.8));
  private final LoggedMechanismLigament2d hopperLigament =
      hopperRoot.append(
          new LoggedMechanismLigament2d(
              "hopper", Units.inchesToMeters(14), 32, 4, new Color8Bit(Color.kGray)));

  private final LoggedMechanismRoot2d climbRoot =
      mech2d.getRoot("climb_base", Units.inchesToMeters(24 + 12.75), Units.inchesToMeters(15.8125));
  private final LoggedMechanismLigament2d climbLigament =
      climbRoot
          .append(
              new LoggedMechanismLigament2d(
                  "climb", Units.inchesToMeters(10), 274.914, 4, new Color8Bit(Color.kRoyalBlue)))
          .append(
              new LoggedMechanismLigament2d(
                  "climb_barb",
                  Units.inchesToMeters(8.5),
                  270,
                  4,
                  new Color8Bit(Color.kRoyalBlue)));
  private final LoggedMechanismLigament2d climbTargetLigament =
      climbRoot
          .append(
              new LoggedMechanismLigament2d(
                  "climb_target",
                  Units.inchesToMeters(10),
                  274.914,
                  2,
                  new Color8Bit(Color.kLightBlue)))
          .append(
              new LoggedMechanismLigament2d(
                  "climb_barb_target",
                  Units.inchesToMeters(8.5),
                  270,
                  2,
                  new Color8Bit(Color.kLightBlue)));

  public Superstructure(
      Hopper hopper,
      Elevator elevator,
      Outtake outtake,
      Gripper gripper,
      Climb climb,
      LED led,
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> velocity,
      Supplier<CoralTarget> coralTarget,
      Supplier<AlgaeTarget> algaeTarget,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      Trigger scoreRequest,
      Trigger coralIntakeRequest,
      Trigger algaeIntakeRequest,
      Trigger preClimbRequest,
      Trigger climbRequest,
      Trigger cancelClimbRequest,
      Trigger homeRequest,
      Trigger superstructureCoastRequest) {
    this.hopper = hopper;
    this.elevator = elevator;
    this.outtake = outtake;
    this.gripper = gripper;
    this.climb = climb;
    this.led = led;

    this.pose = pose;
    this.velocity = velocity;
    this.coralTarget = coralTarget;
    this.algaeTarget = algaeTarget;

    this.driverX = driverX;
    this.driverY = driverY;

    this.scoreRequest = scoreRequest;
    this.coralIntakeRequest = coralIntakeRequest;
    this.algaeIntakeRequest = algaeIntakeRequest;
    this.preClimbRequest = preClimbRequest;
    this.climbRequest = climbRequest;
    this.cancelClimbRequest = cancelClimbRequest;
    this.homeRequest = homeRequest;
    this.superstructureCoastRequest = superstructureCoastRequest;

    for (var state : State.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }

    // IDLE -> CORAL_PREINTAKE
    stateTriggers
        .get(State.IDLE)
        .and(coralIntakeRequest)
        .onTrue(this.forceState(State.CORAL_PREINTAKE));

    // IDLE -> ALGAE_INTAKE_[LEVEL]
    stateTriggers
        .get(State.IDLE)
        .and(algaeIntakeRequest)
        .and(() -> (algaeTarget.get() == AlgaeTarget.A2))
        .onTrue(this.forceState(State.ALGAE_INTAKE_A2));

    stateTriggers
        .get(State.IDLE)
        .and(algaeIntakeRequest)
        .and(() -> (algaeTarget.get() == AlgaeTarget.A3))
        .onTrue(this.forceState(State.ALGAE_INTAKE_A3));

    // IDLE -> CLIMB_PREPULL
    preClimbRequest.onTrue(this.forceState(State.CLIMB_PREPULL));

    // CORAL_PREINTAKE -> CORAL_TRANSFER
    stateTriggers
        .get(State.CORAL_PREINTAKE)
        .whileTrue(elevator.setExtension(ElevatorConstants.intake))
        .whileTrue(outtake.index())
        .whileTrue(hopper.setVoltage(() -> 6.0))
        .and(hopper::getDetected)
        .onTrue(this.forceState(State.CORAL_TRANSFER));

    // CORAL_TRANSFER -> CORAL_READY
    stateTriggers
        .get(State.CORAL_TRANSFER)
        .whileTrue(outtake.index())
        .whileTrue(hopper.setVoltage(() -> 6.0))
        .and(outtake::getDetected)
        .onTrue(
            Commands.parallel(
                outtake.setVoltage(() -> 0),
                hopper.setVoltage(() -> 0),
                this.forceState(State.CORAL_READY)));

    if (Constants.currentMode == Mode.SIM) {
      stateTriggers
          .get(State.CORAL_PREINTAKE)
          .and(
              () ->
                  (AutoAlign.bestLoader(pose.get())
                          .getTranslation()
                          .getDistance(pose.get().getTranslation())
                      < 0.75))
          .onTrue(Commands.sequence(Commands.waitSeconds(1), hopper.setSimDetected(true)));

      stateTriggers
          .get(State.CORAL_TRANSFER)
          .onTrue(
              Commands.sequence(
                  Commands.waitSeconds(0.5), outtake.setSimDetected(true)));
    }

    // CORAL -> IDLE
    stateTriggers
        .get(State.CORAL_READY)
        .or(stateTriggers.get(State.CORAL_PRESCORE))
        .or(stateTriggers.get(State.CORAL_CONFIRM_L1))
        .or(stateTriggers.get(State.CORAL_CONFIRM_L2))
        .or(stateTriggers.get(State.CORAL_CONFIRM_L3))
        .or(stateTriggers.get(State.CORAL_CONFIRM_L4))
        .or(stateTriggers.get(State.CORAL_SCORE_L1))
        .or(stateTriggers.get(State.CORAL_SCORE_L2))
        .or(stateTriggers.get(State.CORAL_SCORE_L3))
        .or(stateTriggers.get(State.CORAL_SCORE_L4))
        .and(() -> !outtake.getDetected())
        .onTrue(this.forceState(State.IDLE));

    // CORAL_OUTTAKE -> IDLE
    stateTriggers
        .get(State.CORAL_OUTTAKE)
        .whileTrue(outtake.setVoltage(OuttakeConstants.L234))
        .and(() -> !outtake.getDetected())
        .and(preClimbRequest.negate())
        .onTrue(this.forceState(State.IDLE));

    // CLIMB_PREPULL -> CLIMB_PULL
    stateTriggers
        .get(State.CLIMB_PREPULL)
        .whileTrue(climb.setPosition(ClimbConstants.ready))
        .and(climbRequest)
        .onTrue(this.forceState(State.CLIMB_PULL));

    stateTriggers
        .get(State.CLIMB_PULL)
        .whileTrue(
            climb
                .setPosition(ClimbConstants.climbed)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // CLIMB_PULL -> CLIMB_PREPULL
    stateTriggers
        .get(State.CLIMB_PULL)
        .and(cancelClimbRequest)
        .onTrue(this.forceState(State.CLIMB_PREPULL));
  }

  /** This file is not a subsystem, so this MUST be called manually. */
  public void periodic() {
    hopperLigament.setColor(
        Math.signum(hopper.getVoltage()) == -1.0
            ? Constants.visualizerNegative
            : Math.signum(hopper.getVoltage()) == 1.0
                ? Constants.visualizerPositive
                : Constants.visualizerNeutral);
    outtakeLigament.setColor(
        Math.signum(hopper.getVoltage()) == -1.0
            ? Constants.visualizerNegative
            : Math.signum(hopper.getVoltage()) == 1.0
                ? Constants.visualizerPositive
                : Constants.visualizerNeutral);
    gripperLigament.setColor(
        Math.signum(gripper.getVoltage()) == -1.0
            ? Constants.visualizerNegative
            : Math.signum(gripper.getVoltage()) == 1.0
                ? Constants.visualizerPositive
                : Constants.visualizerNeutral);

    elevatorLigament.setLength(elevator.getExtensionMeters() + ElevatorConstants.visualizerOffset);
    elevatorTargetLigament.setLength(elevator.getSetpoint() + ElevatorConstants.visualizerOffset);

    climbLigament.setAngle(climb.getPositionRad());
    climbTargetLigament.setAngle(climb.getTargetPositionRad());
    Logger.recordOutput("Superstructure/Mech2d", mech2d);
  }

  private Command forceState(State nextState) {
    return Commands.runOnce(
            () -> {
              System.out.println("Changing state to " + nextState);
              stateTimer.reset();
              this.prevState = this.state;
              this.state = nextState;
            })
        .ignoringDisable(true);
  }
}
