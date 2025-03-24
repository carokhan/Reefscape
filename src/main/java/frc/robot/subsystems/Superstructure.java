package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.Climber;
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
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static enum Target {
    IDLE(0, 0, 0),
    L1(ElevatorConstants.L1, OuttakeConstants.L1, 0),
    L2(ElevatorConstants.L2, OuttakeConstants.L234, 0),
    L3(ElevatorConstants.L3, OuttakeConstants.L234, 0),
    L4(ElevatorConstants.L4, OuttakeConstants.L234, 0),
    AP(ElevatorConstants.AP, 0, GripperConstants.AP),
    A2(ElevatorConstants.A2, 0, GripperConstants.A23),
    A3(ElevatorConstants.A3, 0, GripperConstants.A23),
    AN(ElevatorConstants.AN, 0, GripperConstants.AN);

    public final double elevatorHeight;
    public final double outtakeSpeed;
    public final double gripperSpeed;

    private Target(double elevatorHeight, double outtakeSpeed, double gripperSpeed) {
      this.elevatorHeight = elevatorHeight;
      this.outtakeSpeed = outtakeSpeed;
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
    PRE_CLIMB,
    CLIMB,
    IDLE
  }

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisSpeeds> velocity;
  private final Supplier<Target> target;

  @AutoLogOutput(key = "Superstructure/Score")
  private final Trigger scoreRequest;

  @AutoLogOutput(key = "Superstructure/Coral Intake")
  private final Trigger coralIntakeRequest;
  
  @AutoLogOutput(key = "Superstructure/Reverse Hopper")
  private final Trigger reverseHopperRequest;
  
  @AutoLogOutput(key = "Superstructure/Algae Intake")
  private final Trigger algaeIntakeRequest;
  
  @AutoLogOutput(key = "Superstructure/Climb")
  private final Trigger climbRequest;

  @AutoLogOutput(key = "Superstructure/End Climb")
  private final Trigger endClimbRequest;

  @AutoLogOutput(key = "Superstructure/Homing Request")
  private final Trigger homeRequest;

  private State state = State.IDLE;
  private State prevState = State.IDLE;
  private Map<State, Trigger> stateTriggers = new HashMap<State, Trigger>();

  private final Hopper hopper;
  private final Elevator elevator;
  private final Outtake outtake;
  private final Gripper gripper;
  private final Climber climber;
  private final LED led;

  public Superstructure(
      Hopper hopper,
      Elevator elevator,
      Outtake outtake,
      Gripper gripper,
      Climber climber,
      LED led,
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> velocity,
      Supplier<Target> target,
      Trigger scoreRequest,
      Trigger coralIntakeRequest,
      Trigger reverseHopperRequest,
      Trigger algaeIntakeRequest,
      Trigger climbRequest,
      Trigger endClimbRequest,
      Trigger homeRequest) {
    this.hopper = hopper;
    this.elevator = elevator;
    this.outtake = outtake;
    this.gripper = gripper;
    this.climber = climber;
    this.led = led;

    this.pose = pose;
    this.velocity = velocity;
    this.target = target;

    this.scoreRequest = scoreRequest;
    this.coralIntakeRequest = coralIntakeRequest;
    this.reverseHopperRequest = reverseHopperRequest;
    this.algaeIntakeRequest = algaeIntakeRequest;
    this.climbRequest = climbRequest;
    this.endClimbRequest = endClimbRequest;
    this.homeRequest = homeRequest;

    for (var state: State.values()) {
        stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }
  }

  /** This file is not a subsystem, so this MUST be called manually. */
  public void periodic() {
    Logger.recordOutput("Superstructure/Superstructure State", state);
  }

  
}
