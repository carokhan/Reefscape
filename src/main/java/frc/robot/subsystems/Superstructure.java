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
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.subsystems.autoalign.AutoAlign;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
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
    L2(ElevatorConstants.L2, OuttakeConstants.L23),
    L3(ElevatorConstants.L3, OuttakeConstants.L23),
    L4(ElevatorConstants.L4, OuttakeConstants.L4);

    public final double elevatorHeight;
    public final double outtakeSpeed;

    private CoralTarget(double elevatorHeight, double outtakeSpeed) {
      this.elevatorHeight = elevatorHeight;
      this.outtakeSpeed = outtakeSpeed;
    }
  }

  public enum State {
    CORAL_PREINTAKE,
    CORAL_OUTTAKE,
    CORAL_READY,
    CORAL_CONFIRM,
    ALGAE_INTAKE,
    ALGAE_READY,
    ALGAE_PRESCORE_AP,
    ALGAE_PRESCORE_AN,
    ALGAE_CONFIRM_AP,
    ALGAE_CONFIRM_AN,
    CLIMB_PREPULL,
    CLIMB_PULL,
    IDLE,
    ELEV_MANUAL,
    REV_FUNNEL
  }

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisSpeeds> velocity;

  @AutoLogOutput(key = "Superstructure/Score")
  private final Trigger scoreRequest;

  @AutoLogOutput(key = "Superstructure/Coral Intake")
  private final Trigger coralIntakeRequest;

  @AutoLogOutput(key = "Superstructure/Coral Eject")
  private final Trigger coralEjectRequest;

  @AutoLogOutput(key = "Superstructure/Algae Eject")
  private final Trigger algaeEjectRequest;

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

  @AutoLogOutput(key = "Superstructure/Elevator Manual Request")
  private final Trigger elevManualRequest;

  @AutoLogOutput(key = "Superstructure/Coast Request")
  private final Trigger superstructureCoastRequest;

  @AutoLogOutput(key = "Superstructure/Cancel Request")
  private final Trigger cancelRequest;

  @AutoLogOutput(key = "Superstructure/Reverse Funnel Request")
  private final Trigger revFunnelRequest;

  @AutoLogOutput(key = "Superstructure/State")
  private State state = State.IDLE;

  private State prevState = State.IDLE;

  private Map<State, Trigger> stateTriggers = new HashMap<State, Trigger>();

  private Timer stateTimer = new Timer();

  public final Hopper hopper;
  public final Elevator elevator;
  public final Outtake outtake;
  public final Gripper gripper;
  public final Climb climb;
  public final LED led;

  private final DoubleSupplier driverX;
  private final DoubleSupplier driverY;

  private CoralTarget coralTarget;
  private double algaeTarget;

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
      CoralTarget initCoralTarget,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      Trigger scoreRequest,
      Trigger coralIntakeRequest,
      Trigger coralEjectRequest,
      Trigger algaeEjectRequest,
      Trigger algaeIntakeRequest,
      Trigger algaeNetRequest,
      Trigger algaeProcessRequest,
      Trigger preClimbRequest,
      Trigger climbRequest,
      Trigger cancelClimbRequest,
      Trigger homeRequest,
      Trigger elevManualRequest,
      Trigger superstructureCoastRequest,
      Trigger cancelRequest,
      Trigger revFunnelRequest) {
    this.hopper = hopper;
    this.elevator = elevator;
    this.outtake = outtake;
    this.gripper = gripper;
    this.climb = climb;
    this.led = led;

    this.pose = pose;
    this.velocity = velocity;
    this.coralTarget = initCoralTarget;

    this.driverX = driverX;
    this.driverY = driverY;

    this.scoreRequest = scoreRequest;
    this.coralIntakeRequest = coralIntakeRequest;
    this.coralEjectRequest = coralEjectRequest;
    this.algaeEjectRequest = algaeEjectRequest;
    this.algaeIntakeRequest = algaeIntakeRequest;
    this.algaeNetRequest = algaeNetRequest;
    this.algaeProcessRequest = algaeProcessRequest;
    this.preClimbRequest = preClimbRequest;
    this.climbRequest = climbRequest;
    this.cancelClimbRequest = cancelClimbRequest;
    this.homeRequest = homeRequest;
    this.elevManualRequest = elevManualRequest;
    this.superstructureCoastRequest = superstructureCoastRequest;
    this.cancelRequest = cancelRequest;
    this.revFunnelRequest = revFunnelRequest;

    this.algaeTarget = ElevatorConstants.A3;

    // final Trigger elevatorNetSafety =
    // new Trigger(
    // () ->
    // ((Math.abs(pose.get().getX() - FieldConstants.startingLineX) < 0.625)
    // && (elevator.getExtensionMeters() > 1)));

    for (State state : State.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }

    // elevatorNetSafety.onTrue(
    // Commands.parallel(
    // elevator.setExtension(ElevatorConstants.AP), this.forceState(State.IDLE)));

    cancelRequest.onTrue(
        Commands.parallel(
            elevator
                .homingSequence()
                .andThen(elevator.reset())
                .andThen(elevator.setExtension(ElevatorConstants.intake)),
            outtake.setVoltage(0),
            hopper.setVoltage(0),
            gripper.setVoltage(0),
            this.forceState(State.IDLE)));

    coralEjectRequest.onTrue(
        outtake.setVoltage(OuttakeConstants.L23))
            .onFalse(Commands.parallel(outtake.setVoltage(0), this.forceState(State.IDLE)));

    algaeEjectRequest.onTrue(
            gripper.setVoltage(GripperConstants.AP))
            .onFalse(Commands.parallel(gripper.setVoltage(0), this.forceState(State.IDLE)));

    elevManualRequest
        .onTrue(this.forceState(State.ELEV_MANUAL))
        .onFalse(this.forceState(State.IDLE));

    revFunnelRequest
        .onTrue(this.forceState(State.REV_FUNNEL))
        .onFalse(this.forceState(State.IDLE));

    stateTriggers
        .get(State.ELEV_MANUAL)
        .and(scoreRequest)
        .onTrue(gripper.setVoltage(GripperConstants.AN));

    // IDLE State Transitions (Starts: Robot idle, Ends: Various transitions based
    // on detections and requests)
    stateTriggers
        .get(State.IDLE)
        .onTrue(
            Commands.parallel(
                elevator
                    .homingSequence()
                    .andThen(elevator.reset())
                    .onlyIf(() -> !elevManualRequest.getAsBoolean()),
                gripper.setVoltage(0)));

    stateTriggers
        .get(State.IDLE)
        .and(outtake::getDetected)
        .and(cancelRequest.negate()::getAsBoolean)
        .onTrue(
            Commands.parallel(
                // led.setColor(LEDConstants.Mode.CORAL_READY.color),
                outtake.setVoltage(0), this.forceState(State.CORAL_READY)));

    stateTriggers
        .get(State.IDLE)
        .and(gripper::getDetected)
        .onTrue(
            Commands.parallel(
                gripper.setVoltage(GripperConstants.A23), this.forceState(State.ALGAE_READY)));

    stateTriggers
        .get(State.IDLE)
        .or(stateTriggers.get(State.ALGAE_INTAKE))
        .or(stateTriggers.get(State.ALGAE_READY))
        .and(coralIntakeRequest)
        .or(
            () ->
                (pose.get()
                        .getTranslation()
                        .getDistance(AutoAlign.getBestLoader(pose.get()).getTranslation()))
                    < HopperConstants.enableDistanceToLoader)
        .onTrue(this.forceState(State.CORAL_PREINTAKE));

    stateTriggers
        .get(State.IDLE)
        .or(stateTriggers.get(State.CORAL_PREINTAKE))
        .or(stateTriggers.get(State.CORAL_READY))
        .and(algaeIntakeRequest)
        .and(() -> this.getAlgaeTarget() == ElevatorConstants.A3)
        .onTrue(
            Commands.parallel(
                elevator.setExtension(ElevatorConstants.A3),
                gripper.setVoltage(GripperConstants.A23),
                this.forceState(State.ALGAE_INTAKE)));

    stateTriggers
        .get(State.IDLE)
        .or(stateTriggers.get(State.CORAL_PREINTAKE))
        .or(stateTriggers.get(State.CORAL_READY))
        .and(algaeIntakeRequest)
        .and(() -> this.getAlgaeTarget() == ElevatorConstants.A2)
        .onTrue(
            Commands.parallel(
                elevator.setExtension(ElevatorConstants.A2),
                gripper.setVoltage(GripperConstants.A23),
                this.forceState(State.ALGAE_INTAKE)));

    preClimbRequest.onTrue(this.forceState(State.CLIMB_PREPULL));

    // CORAL State Transitions (Starts: Coral handling, Ends: Scoring or reset)
    stateTriggers
        .get(State.CORAL_PREINTAKE)
        .whileTrue(elevator.setExtension(ElevatorConstants.intake))
        .whileTrue(outtake.index())
        .whileTrue(hopper.setVoltage(() -> HopperConstants.intake))
        .and(() -> outtake.getConnected() ? outtake.getDetected() : true)
        .onTrue(
            Commands.parallel(
                outtake.setVoltage(() -> 0),
                hopper.setVoltage(() -> 0),
                this.forceState(State.CORAL_READY)));

    stateTriggers
        .get(State.CORAL_READY)
        .and(
            () ->
                (pose.get().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center))
                    < ElevatorConstants.reefRaiseDistance))
        .and(() -> (this.getCoralTarget().equals(CoralTarget.L4)))
        .onTrue(
            Commands.parallel(
                elevator.setExtension(ElevatorConstants.L4), this.forceState(State.CORAL_CONFIRM)));

    stateTriggers
        .get(State.CORAL_READY)
        .and(
            () ->
                (pose.get().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center))
                    < ElevatorConstants.reefRaiseDistance))
        .and(() -> (this.getCoralTarget().equals(CoralTarget.L3)))
        .onTrue(
            Commands.parallel(
                elevator.setExtension(ElevatorConstants.L3), this.forceState(State.CORAL_CONFIRM)));

    stateTriggers
        .get(State.CORAL_READY)
        .and(
            () ->
                (pose.get().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center))
                    < ElevatorConstants.reefRaiseDistance))
        .and(() -> (this.getCoralTarget().equals(CoralTarget.L2)))
        .onTrue(
            Commands.parallel(
                elevator.setExtension(ElevatorConstants.L2), this.forceState(State.CORAL_CONFIRM)));

    stateTriggers
        .get(State.CORAL_READY)
        .and(
            () ->
                (pose.get().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center))
                    < ElevatorConstants.reefRaiseDistance))
        .and(() -> (this.getCoralTarget().equals(CoralTarget.L1)))
        .onTrue(
            Commands.parallel(
                elevator.setExtension(ElevatorConstants.L1), this.forceState(State.CORAL_CONFIRM)));

    stateTriggers
        .get(State.CORAL_READY)
        .and(
            () ->
                (pose.get().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center))
                    < ElevatorConstants.reefRaiseDistance))
        .and(
            () ->
                (!this.getCoralTarget().equals(CoralTarget.L1)
                    && !this.getCoralTarget().equals(CoralTarget.L2)
                    && !this.getCoralTarget().equals(CoralTarget.L3)
                    && !this.getCoralTarget().equals(CoralTarget.L4)));

    stateTriggers
        .get(State.CORAL_CONFIRM)
        .and(scoreRequest)
        .onTrue(
            Commands.parallel(
                outtake.setVoltage(OuttakeConstants.targetToCoral.get(coralTarget)),
                Commands.waitUntil(() -> !outtake.getDetected())
                    .andThen(
                        Commands.waitSeconds(ElevatorConstants.confirmTimeout)
                            .andThen(
                                elevator
                                    .setExtension(ElevatorConstants.intake)
                                    .andThen(this.forceState(State.IDLE))))));

    stateTriggers
        .get(State.CORAL_READY)
        .or(stateTriggers.get(State.CORAL_CONFIRM))
        .and(() -> !outtake.getDetected())
        .onTrue(this.forceState(State.IDLE));

    stateTriggers
        .get(State.CORAL_OUTTAKE)
        .whileTrue(outtake.setVoltage(OuttakeConstants.L23))
        .and(() -> !outtake.getDetected())
        .and(preClimbRequest.negate())
        .onTrue(this.forceState(State.IDLE));

    // ALGAE State Transitions (Starts: Algae handling, Ends: Scoring or reset)
    stateTriggers
        .get(State.ALGAE_INTAKE)
        .and(scoreRequest)
        .and(() -> this.getAlgaeTarget() == ElevatorConstants.A3)
        .onTrue(
            elevator
                .setExtension(ElevatorConstants.A3 + .25)
                .onlyWhile(() -> !gripper.getDualDetected())
                .andThen(
                    Commands.waitUntil(
                            () ->
                                (pose.get()
                                        .getTranslation()
                                        .getDistance(AllianceFlipUtil.apply(Reef.center)))
                                    > ElevatorConstants.reefRaiseDistance)
                        .andThen(
                            Commands.parallel(
                                elevator.setExtension(ElevatorConstants.intake),
                                this.forceState(State.ALGAE_READY)))));

    stateTriggers
        .get(State.ALGAE_INTAKE)
        .and(scoreRequest)
        .and(() -> this.getAlgaeTarget() == ElevatorConstants.A2)
        .onTrue(
            Commands.parallel(
                elevator
                    .setExtension(ElevatorConstants.A2 + .25)
                    .onlyWhile(() -> !gripper.getDualDetected())
                    .andThen(
                        Commands.waitUntil(
                                () ->
                                    (pose.get()
                                            .getTranslation()
                                            .getDistance(AllianceFlipUtil.apply(Reef.center)))
                                        > ElevatorConstants.reefRaiseDistance)
                            .andThen(
                                Commands.parallel(
                                    elevator.setExtension(ElevatorConstants.intake),
                                    this.forceState(State.ALGAE_READY))))));

    stateTriggers
        .get(State.ALGAE_READY)
        .or(
            () ->
                (stateTriggers.get(State.ALGAE_INTAKE).getAsBoolean() && gripper.getDualDetected()))
        // .and(
        // () ->
        // (Math.abs(pose.get().getX() - FieldConstants.fieldLength / 2)
        // < ElevatorConstants.netRaiseDistanceUpper)
        // && (Math.abs(pose.get().getX() - FieldConstants.fieldLength / 2)
        // > ElevatorConstants.netRaiseDistanceLower))
        // .and(() -> AllianceFlipUtil.applyY(pose.get().getY()) >
        // FieldConstants.fieldWidth / 2)
        .onTrue(this.forceState(State.ALGAE_CONFIRM_AN));

    stateTriggers
        .get(State.ALGAE_READY)
        .and(algaeProcessRequest)
        .and(
            () ->
                (pose.get()
                        .getTranslation()
                        .getDistance(
                            AllianceFlipUtil.apply(FieldConstants.Processor.centerFace)
                                .getTranslation())
                    < ElevatorConstants.processRaiseDistance))
        .onTrue(
            Commands.parallel(
                elevator.setExtension(ElevatorConstants.AP),
                this.forceState(State.ALGAE_CONFIRM_AP)));

    stateTriggers
        .get(State.ALGAE_CONFIRM_AN)
        .and(scoreRequest)
        .onTrue(
            Commands.parallel(
                    elevator.setExtension(ElevatorConstants.AN),
                    Commands.waitUntil(elevator::isNearExtension)
                        .andThen(gripper.setVoltage(GripperConstants.AN)))
                .onlyWhile(gripper::getDualDetected)
                .andThen(
                    elevator
                        .setExtension(ElevatorConstants.intake)
                        .andThen(this.forceState(State.IDLE))));

    stateTriggers
        .get(State.ALGAE_CONFIRM_AP)
        .and(scoreRequest)
        .onTrue(
            Commands.parallel(
                gripper.setVoltage(GripperConstants.AP),
                Commands.waitSeconds(ElevatorConstants.confirmTimeout)
                    .andThen(
                        elevator
                            .setExtension(ElevatorConstants.intake)
                            .andThen(this.forceState(State.IDLE)))));

    // CLIMB State Transitions (Starts: Climb initiation, Ends: Climbing or reset)
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

    stateTriggers
        .get(State.CLIMB_PULL)
        .and(cancelClimbRequest)
        .onTrue(this.forceState(State.CLIMB_PREPULL));

    // SIM Inputs (Starts: Simulation conditions, Ends: Simulated state triggers)
    if (Constants.currentMode == Mode.SIM) {
      stateTriggers
          .get(State.CORAL_PREINTAKE)
          .and(
              () ->
                  (AutoAlign.getBestLoader(pose.get())
                          .getTranslation()
                          .getDistance(pose.get().getTranslation())
                      < 0.75))
          .onTrue(
              Commands.sequence(
                  Commands.waitSeconds(0.5),
                  Commands.parallel(hopper.setSimDetected(true), outtake.setSimDetected(true))));

      stateTriggers
          .get(State.CORAL_CONFIRM)
          .and(scoreRequest)
          .onTrue(Commands.sequence(Commands.waitSeconds(0.5), outtake.setSimDetected(false)));

      stateTriggers
          .get(State.ALGAE_INTAKE)
          .and(
              () ->
                  (FieldConstants.Reef.algaeIntake[
                          AutoAlign.getBestFace(
                              pose.get(), driverX.getAsDouble(), driverY.getAsDouble())]
                          .getTranslation()
                          .getDistance(pose.get().getTranslation())
                      < 0.75))
          .onTrue(Commands.sequence(Commands.waitSeconds(0.5), gripper.setSimDetected(true)));

      stateTriggers
          .get(State.ALGAE_CONFIRM_AN)
          .and(scoreRequest)
          .onTrue(Commands.sequence(Commands.waitSeconds(0.5), gripper.setSimDetected(false)));
    }
  }

  /** This file is not a subsystem, so this MUST be called manually. */
  public void periodic() {
    algaeTarget =
        FieldConstants.Reef.algaeHeights.get(
            FieldConstants.Reef.centerFaces[
                AutoAlign.getBestFace(pose.get(), driverX.getAsDouble(), driverY.getAsDouble())]);

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

    Logger.recordOutput("Superstructure/CoralTarget", coralTarget);
    Logger.recordOutput("Superstructure/AlgaeTarget", algaeTarget);

    Logger.recordOutput(
        "AutoAlign/DistanceToReef",
        pose.get().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center)));
  }

  public Command forceState(State nextState) {
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
    return this.state == State.CORAL_CONFIRM || this.state == State.CORAL_READY;
  }

  public Command setCoralTarget(CoralTarget coralTarget) {
    return Commands.runOnce(
        () -> {
          this.coralTarget = coralTarget;
        });
  }

  public CoralTarget getCoralTarget() {
    // Commands.print(this.coralTarget);
    return this.coralTarget;
  }

  public double getAlgaeTarget() {
    // Commands.print(this.coralTarget);
    return this.algaeTarget;
  }
}
