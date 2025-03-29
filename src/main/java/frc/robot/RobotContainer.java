// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.AlgaeTarget;
import frc.robot.subsystems.Superstructure.CoralTarget;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOReal;
import frc.robot.subsystems.led.LEDIOSim;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOSim;
import frc.robot.subsystems.outtake.OuttakeIOTalonFX;
import frc.robot.subsystems.proximity.ProximityIO;
import frc.robot.subsystems.proximity.ProximityIORedux;
import frc.robot.subsystems.proximity.ProximityIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.CommandXboxControllerSubsystem;
import java.util.Arrays;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;

  // Subsystems
  public final Drive drive;
  public final Hopper hopper;
  public final Elevator elevator;
  public final Outtake outtake;
  public final Gripper gripper;
  public final Climb climb;
  public final LED led;
  public final Vision vision;

  public static Superstructure superstructure;

  // Controller
  private final CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  private final AutoFactory autoFactory;
  private final LoggedDashboardChooser<Command> autoChooser;

  private CoralTarget coralTarget = CoralTarget.L4;
  private AlgaeTarget algaeTarget = AlgaeTarget.AN;

  private boolean superstructureCoastOverride = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (ROBOT_TYPE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        hopper =
            new Hopper(
                new HopperIOSpark(),
                // new ProximityIOGrapple(HopperConstants.laser, null,
                new ProximityIO() {});
        elevator = new Elevator(new ElevatorIOTalonFX());
        outtake =
            new Outtake(
                new OuttakeIOTalonFX(),
                new ProximityIORedux(
                    OuttakeConstants.canandcolor, OuttakeConstants.proximityThreshold),
                new ProximityIO() {
                  {
                  }
                });
        gripper =
            new Gripper(
                new GripperIOTalonFX(),
                // new ProximityIOGrapple(GripperConstants.laser, null,
                new ProximityIO() {});
        climb = new Climb(new ClimbIOTalonFX());
        led = new LED(new LEDIOReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.CAM_FL_NAME,
                    VisionConstants.robotToLeftCam,
                    drive::getRotation),
                new VisionIOPhotonVision(
                    VisionConstants.CAM_FR_NAME,
                    VisionConstants.robotToRightCam,
                    drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        hopper = new Hopper(new HopperIOSim(), new ProximityIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim(), new ProximityIOSim(), new ProximityIOSim());
        gripper = new Gripper(new GripperIOSim(), new ProximityIOSim());
        climb = new Climb(new ClimbIOSim());
        led = new LED(new LEDIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAM_FL_NAME, VisionConstants.robotToLeftCam, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAM_FR_NAME, VisionConstants.robotToRightCam, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        hopper = new Hopper(new HopperIO() {}, new ProximityIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        outtake = new Outtake(new OuttakeIO() {}, new ProximityIO() {}, new ProximityIO() {});
        gripper = new Gripper(new GripperIO() {}, new ProximityIO() {});
        climb = new Climb(new ClimbIO() {});
        led = new LED(new LEDIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    superstructure =
        new Superstructure(
            hopper,
            elevator,
            outtake,
            gripper,
            climb,
            led,
            drive::getPose,
            drive::getVelocityFieldRelative,
            () -> coralTarget,
            () -> algaeTarget,
            driver.rightTrigger(),
            driver.leftTrigger(),
            driver.leftTrigger(),
            driver.a(),
            driver.rightTrigger(),
            driver.rightBumper(),
            operator.leftTrigger(),
            operator.leftBumper());

    autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            (sample, isStart) -> {
              Logger.recordOutput(
                  "ActiveTrajectory",
                  Arrays.stream(sample.getPoses())
                      .map(AllianceFlipUtil::apply)
                      .toArray(Pose2d[]::new));
            });

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Choreo Auto Chooser");

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    RobotModeTriggers.autonomous()
        .whileTrue(Commands.defer(() -> autoChooser.get().asProxy(), Set.of()));

    climb.setCoastOverride(() -> superstructureCoastOverride);

    driver.setDefaultCommand(driver.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> OperatorConstants.deadband,
            () -> 1));
    driver
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driver.povRight().onTrue(Commands.runOnce(() -> climb.resetEncoder()));

    operator
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralTarget = CoralTarget.L4;
                  algaeTarget = AlgaeTarget.AN;
                }));
    operator
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralTarget = CoralTarget.L3;
                }));
    operator
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralTarget = CoralTarget.L2;
                }));
    operator
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralTarget = CoralTarget.L1;
                  algaeTarget = AlgaeTarget.AP;
                }));

    operator
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (DriverStation.isDisabled()) {
                        superstructureCoastOverride = true;
                      }
                    })
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      superstructureCoastOverride = false;
                    })
                .ignoringDisable(true));
    RobotModeTriggers.disabled()
        .onFalse(
            Commands.runOnce(
                    () -> {
                      superstructureCoastOverride = false;
                    })
                .ignoringDisable(true));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime()
                        <= Math.round(OperatorConstants.endgameAlert1Seconds))
        .onTrue(driver.rumbleCmd(0.5, 0.5).withTimeout(0.5).withName("Controller Endgame Alert 1"));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime()
                        <= Math.round(OperatorConstants.endgameAlert2Seconds))
        .onTrue(
            driver
                .rumbleCmd(1.0, 1.0)
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9)
                .withName("Controller Endgame Alert 2")); // Rumble three times
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
