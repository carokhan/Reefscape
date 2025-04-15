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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.Reef;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CoralTarget;
import frc.robot.subsystems.autoalign.AutoAlign;
import frc.robot.subsystems.autoalign.AutoAlign.IntakeLocation;
import frc.robot.subsystems.autoalign.AutoAlignConstants;
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
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOCandle;
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
  private final CommandXboxController operator = new CommandXboxController(1);

  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoRoutines autoRoutines;

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
                new HopperIOTalonFX(),
                // new ProximityIOGrapple(HopperConstants.laser, null,
                new ProximityIO() {});
        elevator = new Elevator(new ElevatorIOTalonFX());
        outtake =
            new Outtake(
                new OuttakeIOTalonFX(),
                new ProximityIORedux(
                    OuttakeConstants.canandcolor,
                    OuttakeConstants.proximityThreshold,
                    OuttakeConstants.period),
                new ProximityIO() {
                  {
                  }
                });
        gripper =
            new Gripper(
                new GripperIOTalonFX(),
                // new ProximityIOGrapple(GripperConstants.laser, null,
                new ProximityIORedux(
                    GripperConstants.laser,
                    GripperConstants.proximityThreshold,
                    GripperConstants.period));
        climb = new Climb(new ClimbIOTalonFX());
        led = new LED(new LEDIOCandle());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.CAM_FR_NAME,
                    VisionConstants.robotToLeftCam,
                    drive::getRotation),
                new VisionIOPhotonVision(
                    VisionConstants.CAM_FL_NAME,
                    VisionConstants.robotToRightCam,
                    drive::getRotation));
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new
        // VisionIO() {});

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
        hopper = new Hopper(new HopperIOSim(), new ProximityIOSim(0.1));
        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim(), new ProximityIOSim(0.1), new ProximityIO() {});
        gripper = new Gripper(new GripperIOSim(), new ProximityIOSim(0.1));
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
            CoralTarget.L4,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -operator.getRightY(),
            driver.rightTrigger(),
            driver
                .leftTrigger()
                .and(
                    () ->
                        (AutoAlign.closerIntake(
                                drive.getPose(), -driver.getLeftY(), -driver.getLeftX())
                            == IntakeLocation.SOURCE)),
            driver.b(),
            driver.x(),
            driver
                .leftTrigger()
                .and(
                    () ->
                        (AutoAlign.closerIntake(
                                drive.getPose(), -driver.getLeftY(), -driver.getLeftX())
                            == IntakeLocation.REEF)),
            driver.povUp(),
            driver.povDown(),
            driver.a(),
            driver.rightTrigger(),
            driver.rightBumper(),
            operator.leftTrigger(),
            operator.rightTrigger(),
            operator.leftBumper(),
            driver.povLeft(),
            driver.povRight());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Choreo Auto Chooser");
    autoRoutines = new AutoRoutines(drive, superstructure);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption("Elevator static",
    // elevator.staticCharacterization(1.0));

    autoChooser.addOption("LM to H", autoRoutines.LMtoH());
    autoChooser.addOption("RO to E", autoRoutines.ROtoE());
    autoChooser.addOption("LO to I", autoRoutines.LOtoI());
    autoChooser.addOption("LO to J", autoRoutines.LOtoJ());
    autoChooser.addOption("LO to L", autoRoutines.LOtoL());
    autoChooser.addOption("RO to C", autoRoutines.ROtoC());
    autoChooser.addOption("LM to barge", autoRoutines.LMtobarge());

    // RobotModeTriggers.autonomous()
    // .whileTrue(Commands.defer(() -> autoChooser.get().asProxy(), Set.of()));

    climb.setCoastOverride(() -> superstructureCoastOverride);

    driver.setDefaultCommand(driver.rumbleCmd(0.0, 0.0));

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
    driver // TODO: TEST IF YOU CAN RESET GYRO ON INIT.
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driver
        .povRight()
        .onTrue(
            Commands.parallel(
                superstructure.hopper.setVoltage(-7), superstructure.outtake.setVoltage(-7)))
        .onFalse(
            Commands.parallel(
                superstructure.hopper.setVoltage(6), superstructure.outtake.setVoltage(0)));

    driver
        .rightBumper()
        .or(driver.leftBumper())
        .and(outtake::getDetected)
        .whileTrue(
            Commands.parallel(
                AutoAlign.autoAimWithIntermediatePose(
                        drive,
                        () -> {
                          int bestFace =
                              AutoAlign.getBestFace(
                                  drive.getPose(), -driver.getLeftY(), -driver.getLeftX());
                          Pose2d selected =
                              AllianceFlipUtil.apply(
                                  (driver.leftBumper().getAsBoolean())
                                      ? (Reef.robotLeft[bestFace])
                                      : Reef.robotRight[bestFace]);
                          Logger.recordOutput("AutoAlign/Active", selected);
                          return selected;
                        },
                        // Keeps the robot off the reef wall until it's aligned side-side
                        new Transform2d(
                            AutoAlignConstants.offsetReefKeepOff
                                * Math.signum(drive.getVelocityFieldRelative().vyMetersPerSecond),
                            0.0,
                            Rotation2d.kZero))
                    .andThen(drive.stopWithXCmd()),
                Commands.waitUntil(
                        () ->
                            AutoAlign.isInToleranceCoral(
                                drive.getPose(), -driver.getLeftY(), -driver.getLeftX()))
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    driver
        .leftTrigger()
        .and(() -> !outtake.getDetected())
        .and(
            () ->
                gripper.getDetected()
                    || (AutoAlign.closerIntake(
                            drive.getPose(), -driver.getLeftY(), -driver.getLeftX())
                        == IntakeLocation.SOURCE))
        .whileTrue(
            Commands.parallel(
                AutoAlign.autoAimWithIntermediatePose(
                        drive,
                        () -> {
                          Pose2d bestLoader = AutoAlign.getBestLoader(drive.getPose());
                          Pose2d selected = bestLoader;
                          Logger.recordOutput("AutoAlign/Active", selected);
                          return selected;
                        },
                        // Keeps the robot off the reef wall until it's aligned side-side
                        new Transform2d(0.0, 0.0, Rotation2d.kZero))
                    .andThen(drive.stopWithXCmd()),
                Commands.waitUntil(
                        () ->
                            AutoAlign.isInTolerance(
                                drive.getPose(), AutoAlign.getBestLoader(drive.getPose())))
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    driver
        .leftTrigger()
        .and(() -> !gripper.getDetected())
        .and(
            () ->
                outtake.getDetected()
                    || (AutoAlign.closerIntake(
                            drive.getPose(), -driver.getLeftY(), -driver.getLeftX())
                        == IntakeLocation.REEF))
        .whileTrue(
            Commands.parallel(
                AutoAlign.autoAimWithIntermediatePose(
                        drive,
                        () -> {
                          int bestFace =
                              AutoAlign.getBestFace(
                                  drive.getPose(), -driver.getLeftY(), -driver.getLeftX());
                          Pose2d selected = AllianceFlipUtil.apply(Reef.algaeIntake[bestFace]);
                          Logger.recordOutput("AutoAlign/Active", selected);
                          return selected;
                        },
                        // Keeps the robot off the reef wall until it's aligned side-side
                        new Transform2d(
                            AutoAlignConstants.offsetReefKeepOff
                                * Math.signum(drive.getVelocityFieldRelative().vyMetersPerSecond),
                            0.0,
                            Rotation2d.kZero))
                    .andThen(drive.stopWithXCmd()),
                Commands.waitUntil(
                        () ->
                            AutoAlign.isInToleranceCoral(
                                drive.getPose(), -driver.getLeftY(), -driver.getLeftX()))
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    driver
        .povUp()
        .and(gripper::getDetected)
        .whileTrue(
            Commands.parallel(
                AutoAlign.autoAimWithIntermediatePose(
                        drive,
                        () -> AllianceFlipUtil.apply(FieldConstants.Barge.net),
                        // Keeps the robot off the reef wall until it's aligned side-side
                        new Transform2d(0.0, 0.0, Rotation2d.kZero),
                        new Constraints(
                            AutoAlignConstants.maxLinearSpeed / 2,
                            AutoAlignConstants.maxLinearAccel / 2))
                    .andThen(drive.stopWithXCmd()),
                Commands.waitUntil(
                        () ->
                            AutoAlign.isInTolerance(
                                AllianceFlipUtil.apply(FieldConstants.Barge.net), drive.getPose()))
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    driver
        .povDown()
        .and(gripper::getDetected)
        .whileTrue(
            Commands.parallel(
                AutoAlign.autoAimWithIntermediatePose(
                        drive,
                        () -> AllianceFlipUtil.apply(FieldConstants.Processor.robotProcess),
                        // Keeps the robot off the reef wall until it's aligned side-side
                        new Transform2d(0.0, 0.0, Rotation2d.kZero),
                        new Constraints(
                            AutoAlignConstants.maxLinearSpeed / 2,
                            AutoAlignConstants.maxLinearAccel / 2))
                    .andThen(drive.stopWithXCmd()),
                Commands.waitUntil(
                        () ->
                            AutoAlign.isInTolerance(
                                AllianceFlipUtil.apply(FieldConstants.Processor.robotProcess),
                                drive.getPose()))
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    operator.y().onTrue(superstructure.setCoralTarget(CoralTarget.L4));
    operator.x().onTrue(superstructure.setCoralTarget(CoralTarget.L3));
    operator.b().onTrue(superstructure.setCoralTarget(CoralTarget.L2));
    operator.a().onTrue(superstructure.setCoralTarget(CoralTarget.L1));

    operator
        .rightTrigger()
        .and(operator.y())
        .onTrue(superstructure.elevator.setExtension((ElevatorConstants.L4)));
    operator
        .rightTrigger()
        .and(operator.x())
        .onTrue(superstructure.elevator.setExtension((ElevatorConstants.L3)));
    operator
        .rightTrigger()
        .and(operator.b())
        .onTrue(superstructure.elevator.setExtension((ElevatorConstants.L2)));
    operator
        .rightTrigger()
        .and(operator.a())
        .onTrue(superstructure.elevator.setExtension((ElevatorConstants.L1)));

    operator.leftTrigger().onTrue(elevator.homingSequence().andThen(elevator.reset()));
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
    operator
        .rightTrigger()
        .and(operator.a())
        .onTrue(superstructure.elevator.setExtension(ElevatorConstants.L1));

    operator
        .rightTrigger()
        .and(operator.b())
        .onTrue(superstructure.elevator.setExtension(ElevatorConstants.L2));

    operator
        .rightTrigger()
        .and(operator.x())
        .onTrue(superstructure.elevator.setExtension(ElevatorConstants.L3));

    operator
        .rightTrigger()
        .and(operator.y())
        .onTrue(superstructure.elevator.setExtension(ElevatorConstants.L4));

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

    // SmartDashboard.putData("Run Elevator Sysid", elevator.runSysid());
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
