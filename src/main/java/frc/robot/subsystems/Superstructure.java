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
import frc.robot.FieldConstants.Reef;
import frc.robot.subsystems.autoalign.AutoAlign;
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
import frc.robot.util.AllianceFlipUtil;
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
    CORAL_CONFIRM,
    ALGAE_INTAKE,
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

  @AutoLogOutput(key = "Superstructure/Algae Net")
  private final Trigger algaeNetRequest;

  @AutoLogOutput(key = "Superstructure/Algae Process")
  private final Trigger algaeProcessRequest;

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
      Trigger algaeNetRequest,
      Trigger algaeProcessRequest,
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
    this.algaeNetRequest = algaeNetRequest;
    this.algaeProcessRequest = algaeProcessRequest;
    this.preClimbRequest = preClimbRequest;
    this.climbRequest = climbRequest;
    this.cancelClimbRequest = cancelClimbRequest;
    this.homeRequest = homeRequest;
    this.superstructureCoastRequest = superstructureCoastRequest;

    for (State state : State.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }

    // IDLE -> CORAL_READY
    stateTriggers
        .get(State.IDLE)
        .and(outtake::getDetected)
        .onTrue(Commands.parallel(outtake.setVoltage(0), this.forceState(State.CORAL_READY)));

    // IDLE -> ALGAE_READY
    stateTriggers
        .get(State.IDLE)
        .and(gripper::getDetected)
        .onTrue(this.forceState(State.ALGAE_READY));

    // IDLE -> CORAL_PREINTAKE
    stateTriggers
        .get(State.IDLE)
        .and(coralIntakeRequest)
        .or(
            () ->
                (pose.get()
                        .getTranslation()
                        .getDistance(AutoAlign.bestLoader(pose.get()).getTranslation()))
                    < 0.3)
        .onTrue(this.forceState(State.CORAL_PREINTAKE));

    // IDLE -> ALGAE_INTAKE
    stateTriggers
        .get(State.IDLE)
        .or(stateTriggers.get(State.CORAL_READY))
        .and(algaeIntakeRequest)
        .onTrue(
            Commands.parallel(
                elevator.setExtension(ElevatorConstants.targetToAlgae.get(algaeTarget.get())),
                elevator.setExtension(ElevatorConstants.targetToAlgae.get(algaeTarget.get())),
                gripper.setVoltage(GripperConstants.A23),
                this.forceState(State.ALGAE_INTAKE)));

    // ALGAE_INTAKE -> ALGAE_READY
    stateTriggers
        .get(State.ALGAE_INTAKE)
        .and(gripper::getDetected)
        .onTrue(
            Commands.parallel(
                gripper.setVoltage(GripperConstants.hold),
                elevator.setExtension(ElevatorConstants.intake),
                this.forceState(State.ALGAE_READY)));

    // ALGAE_READY -> ALGAE_CONFIRM
    // stateTriggers.get(State.ALGAE_READY).and(
    //     () -> (pose.get().getTranslation().getDistance(AllianceFlipUtil.apply(algaeTarget.get()))
    // < 1.5))

    // IDLE -> CLIMB_PREPULL
    preClimbRequest.onTrue(this.forceState(State.CLIMB_PREPULL));

    // CORAL_PREINTAKE -> CORAL_TRANSFER
    stateTriggers
        .get(State.CORAL_PREINTAKE)
        .whileTrue(elevator.setExtension(ElevatorConstants.intake))
        .whileTrue(outtake.index())
        .whileTrue(hopper.setVoltage(() -> 6.0))
        .and(
            () -> {
              if (hopper.getConnected()) {
                return hopper.getDetected();
              } else {
                return true;
              }
            })
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

    // CORAL_READY -> CORAL_CONFIRM_[LEVEL]
    stateTriggers
        .get(State.CORAL_READY)
        .and(
            () ->
                (pose.get().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center))
                    < 1.5))
        .onTrue(
            Commands.parallel(
                elevator.setExtension(ElevatorConstants.targetToCoral.get(coralTarget.get())),
                this.forceState(State.CORAL_CONFIRM)));

    stateTriggers
        .get(State.CORAL_CONFIRM)
        .and(scoreRequest)
        .onTrue(
            Commands.parallel(
                outtake.setVoltage(OuttakeConstants.targetToCoral.get(coralTarget.get())),
                Commands.waitSeconds(1)
                    .andThen(
                        elevator
                            .setExtension(ElevatorConstants.intake)
                            .andThen(this.forceState(State.IDLE)))));

    // SIM INPUTS
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
          .onTrue(Commands.sequence(Commands.waitSeconds(0.5), outtake.setSimDetected(true)));
    }

    // CORAL -> IDLE
    stateTriggers
        .get(State.CORAL_READY)
        .or(stateTriggers.get(State.CORAL_PRESCORE))
        .or(stateTriggers.get(State.CORAL_CONFIRM))
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

    AutoAlign.bestFace(pose.get(), driverX.getAsDouble(), driverY.getAsDouble());

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
    elevatorTargetLigament.setLength(
        elevator.getTargetExtensionMeters() + ElevatorConstants.visualizerOffset);

    climbLigament.setAngle(climb.getPositionRad());
    climbTargetLigament.setAngle(climb.getTargetPositionRad());
    Logger.recordOutput("Superstructure/Mech2d", mech2d);

    Logger.recordOutput("Superstructure/CoralTarget", coralTarget.get());
    Logger.recordOutput("Superstructure/AlgaeTarget", algaeTarget.get());

    Logger.recordOutput(
        "AutoAlign/DistanceToReef",
        pose.get().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center)));
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

  public boolean isCoral() {
    return this.state == State.CORAL_CONFIRM
        || this.state == State.CORAL_PRESCORE
        || this.state == State.CORAL_READY;
  }
}
