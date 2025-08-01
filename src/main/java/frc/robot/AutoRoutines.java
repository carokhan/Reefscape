package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.autoalign.AutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoRoutines {
  private final Drive drive;
  private final Superstructure superstructure;
  private final AutoFactory factory;

  public AutoRoutines(Drive drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
    factory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive.choreoDriveController(),
            true,
            drive,
            (traj, edge) -> {
              Logger.recordOutput(
                  "Choreo/Active Traj",
                  DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Blue)
                      ? traj.getPoses()
                      : traj.flipped().getPoses());
            });
  }

  public AutoFactory getFactory() {
    return factory;
  }

  public Command getNoneAuto() {
    final var routine = factory.newRoutine("None");
    routine.active().onTrue(Commands.print("Running empty auto."));
    return routine.cmd();
  }

  public Command LMtoH() {
    superstructure.outtake.setSimDetected(true);
    final var routine = factory.newRoutine("LM to H");
    final var score = routine.trajectory("LMtoH");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command RMtoG() {
    superstructure.outtake.setSimDetected(true);
    final var routine = factory.newRoutine("RM to G");
    final var score = routine.trajectory("RMtoG");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command ROtoE() {
    superstructure.outtake.setSimDetected(true);
    final var routine = factory.newRoutine("RO to E");
    final var score = routine.trajectory("ROtoE");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command LOtoI() {
    superstructure.outtake.setSimDetected(true);
    final var routine = factory.newRoutine("LO to I");
    final var score = routine.trajectory("LOtoI");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(5).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command LOfasttoJ() {
    superstructure.outtake.setSimDetected(true);
    final var routine = factory.newRoutine("LOfast to J");
    final var score = routine.trajectory("LOfasttoJ");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.0625))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(5).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command ROfasttoE() {
    superstructure.outtake.setSimDetected(true);
    final var routine = factory.newRoutine("ROfast to E");
    final var score = routine.trajectory("ROfasttoE");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(5).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command ItoPLO() {
    final var routine = factory.newRoutine("I to PLO");
    final var intake = routine.trajectory("ItoPLO");
    routine.active().whileTrue(Commands.sequence(intake.resetOdometry(), intake.cmd()));

    routine
        .observe(intake.done())
        .onTrue(
            Commands.deadline(
                    Commands.waitUntil(superstructure.outtake::getDetected),
                    AutoAlign.translateToPose(
                        drive, () -> AutoAlign.getBestLoader(drive.getPose())))
                .andThen(
                    Commands.parallel(
                        superstructure.hopper.setVoltage(0),
                        superstructure.outtake.setVoltage(0))));
    return routine.cmd();
  }

  public Command PLOtoK() {
    final var routine = factory.newRoutine("PLO to K");
    final var score = routine.trajectory("PLOtoK");

    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(3).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command KtoPLO() {
    final var routine = factory.newRoutine("K to PLO");
    final var intake = routine.trajectory("KtoPLO");

    routine.active().whileTrue(Commands.sequence(intake.resetOdometry(), intake.cmd()));

    routine
        .observe(intake.done())
        .onTrue(
            Commands.deadline(
                    Commands.waitUntil(superstructure.outtake::getDetected),
                    AutoAlign.translateToPose(
                        drive, () -> AutoAlign.getBestLoader(drive.getPose())))
                .andThen(
                    Commands.parallel(
                        superstructure.hopper.setVoltage(0),
                        superstructure.outtake.setVoltage(0))));
    return routine.cmd();
  }

  public Command PLOtoL() {
    final var routine = factory.newRoutine("PLO to L");
    final var score = routine.trajectory("PLOtoL");

    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(2).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command LtoPLO() {
    final var routine = factory.newRoutine("L to PLO");
    final var intake = routine.trajectory("LtoPLO");

    routine.active().whileTrue(Commands.sequence(intake.resetOdometry(), intake.cmd()));

    routine
        .observe(intake.done())
        .onTrue(
            Commands.deadline(
                    Commands.waitUntil(superstructure.outtake::getDetected),
                    AutoAlign.translateToPose(
                        drive, () -> AutoAlign.getBestLoader(drive.getPose())))
                .andThen(
                    Commands.parallel(
                        superstructure.hopper.setVoltage(0),
                        superstructure.outtake.setVoltage(0))));
    return routine.cmd();
  }

  public Command LOtoL() {
    return Commands.sequence(
        this.LOtoI().onlyWhile(superstructure.outtake::getDetected),
        this.ItoPLO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        this.PLOtoK().onlyWhile(superstructure.outtake::getDetected),
        this.KtoPLO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        this.PLOtoL().onlyWhile(superstructure.outtake::getDetected),
        this.LtoPLO());
  }

  public Command JtoPLO() {
    final var routine = factory.newRoutine("J to PLO");
    final var intake = routine.trajectory("JtoPLO");
    routine.active().whileTrue(Commands.sequence(intake.resetOdometry(), intake.cmd()));

    routine
        .observe(intake.done())
        .onTrue(
            Commands.deadline(
                    Commands.waitUntil(superstructure.outtake::getDetected),
                    AutoAlign.translateToPose(
                        drive, () -> AutoAlign.getBestLoader(drive.getPose())))
                .andThen(
                    Commands.parallel(
                        superstructure.hopper.setVoltage(0),
                        superstructure.outtake.setVoltage(0))));
    return routine.cmd();
  }

  public Command PLOtoA4() {
    final var routine = factory.newRoutine("PLO to A4");
    final var score = routine.trajectory("PLOtoA");

    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(3).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command PLOtoA2() {
    final var routine = factory.newRoutine("PLO to A2");
    final var score = routine.trajectory("PLOtoA");

    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                superstructure.elevator.setExtension(ElevatorConstants.L2),
                Commands.waitUntil(this::atScore2)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L23)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(3).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command PROtoB4() {
    final var routine = factory.newRoutine("PRO to B4");
    final var score = routine.trajectory("PROtoB");

    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(3).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command PROtoB2() {
    final var routine = factory.newRoutine("PRO to B2");
    final var score = routine.trajectory("PROtoB");

    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                superstructure.elevator.setExtension(ElevatorConstants.L2),
                Commands.waitUntil(this::atScore2)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L23)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(3).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command LOtoL4() {
    return Commands.sequence(
        this.LOfasttoJ().onlyWhile(superstructure.outtake::getDetected),
        this.JtoPLO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        this.PLOtoK().onlyWhile(superstructure.outtake::getDetected),
        this.KtoPLO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        this.PLOtoL().onlyWhile(superstructure.outtake::getDetected),
        this.LtoPLO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        Commands.either(this.PLOtoA4(), this.PLOtoA2(), () -> DriverStation.getMatchTime() > 2.0));
  }

  public Command ROtoF() {
    superstructure.outtake.setSimDetected(true);
    final var routine = factory.newRoutine("RO to F");
    final var score = routine.trajectory("ROtoF");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(5).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command FtoPRO() {
    final var routine = factory.newRoutine("F to PRO");
    final var intake = routine.trajectory("FtoPRO");
    routine.active().whileTrue(Commands.sequence(intake.resetOdometry(), intake.cmd()));

    routine
        .observe(intake.done())
        .onTrue(
            Commands.deadline(
                    Commands.waitUntil(superstructure.outtake::getDetected),
                    AutoAlign.translateToPose(
                        drive, () -> AutoAlign.getBestLoader(drive.getPose())))
                .andThen(
                    Commands.parallel(
                        superstructure.hopper.setVoltage(0),
                        superstructure.outtake.setVoltage(0))));
    return routine.cmd();
  }

  public Command PROtoD() {
    final var routine = factory.newRoutine("PRO to D");
    final var score = routine.trajectory("PROtoD");

    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(3).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command DtoPRO() {
    final var routine = factory.newRoutine("D to PRO");
    final var intake = routine.trajectory("DtoPRO");

    routine.active().whileTrue(Commands.sequence(intake.resetOdometry(), intake.cmd()));

    routine
        .observe(intake.done())
        .onTrue(
            Commands.deadline(
                    Commands.waitUntil(superstructure.outtake::getDetected),
                    AutoAlign.translateToPose(
                        drive, () -> AutoAlign.getBestLoader(drive.getPose())))
                .andThen(
                    Commands.parallel(
                        superstructure.hopper.setVoltage(0),
                        superstructure.outtake.setVoltage(0))));
    return routine.cmd();
  }

  public Command PROtoC() {
    final var routine = factory.newRoutine("PRO to C");
    final var score = routine.trajectory("PROtoC");

    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));

    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        // AllianceFlipUtil.apply(
                        // FieldConstants.Reef.branchPositions2d.get(2).get(ReefLevel.L4))
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command CtoPRO() {
    final var routine = factory.newRoutine("C to PRO");
    final var intake = routine.trajectory("CtoPRO");

    routine.active().whileTrue(Commands.sequence(intake.resetOdometry(), intake.cmd()));

    routine
        .observe(intake.done())
        .onTrue(
            Commands.deadline(
                    Commands.waitUntil(superstructure.outtake::getDetected),
                    AutoAlign.translateToPose(
                        drive, () -> AutoAlign.getBestLoader(drive.getPose())))
                .andThen(
                    Commands.parallel(
                        superstructure.hopper.setVoltage(0),
                        superstructure.outtake.setVoltage(0))));
    return routine.cmd();
  }

  public Command ROtoC() {
    return Commands.sequence(
        this.ROtoF().onlyWhile(superstructure.outtake::getDetected),
        this.FtoPRO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        this.PROtoD().onlyWhile(superstructure.outtake::getDetected),
        this.DtoPRO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        this.PROtoC().onlyWhile(superstructure.outtake::getDetected),
        this.CtoPRO());
  }

  public Command ROtoC4() {
    return Commands.sequence(
        this.ROfasttoE().onlyWhile(superstructure.outtake::getDetected),
        this.FtoPRO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        this.PROtoD().onlyWhile(superstructure.outtake::getDetected),
        this.DtoPRO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        this.PROtoC().onlyWhile(superstructure.outtake::getDetected),
        this.CtoPRO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        Commands.either(this.PROtoB4(), this.PROtoB2(), () -> DriverStation.getMatchTime() > 2.0));
  }

  public Command LOtoJ() {
    superstructure.outtake.setSimDetected(true);
    final var routine = factory.newRoutine("LO to J");
    final var score = routine.trajectory("LOtoJ");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));
    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore4)
                    .andThen(Commands.waitSeconds(0.03125))
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command HtoI() {
    final var routine = factory.newRoutine("H to I");
    final var intake = routine.trajectory("HtoI");
    routine.active().whileTrue(Commands.sequence(intake.resetOdometry(), intake.cmd()));
    routine
        .observe(intake.done())
        .onTrue(
            Commands.parallel(
                Commands.parallel(
                    superstructure.elevator.setExtension(ElevatorConstants.A3),
                    superstructure.gripper.setVoltage(GripperConstants.A23)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            Reef.centerFaces[AutoAlign.getBestFace(drive.getPose(), 0, 0)]
                                .rotateAround(
                                    Reef.centerFaces[AutoAlign.getBestFace(drive.getPose(), 0, 0)]
                                        .getTranslation(),
                                    Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command Itobarge() {
    final var routine = factory.newRoutine("I to barge");
    final var score = routine.trajectory("Itobarge");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));
    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                    Commands.waitUntil(this::atBarge)
                        .andThen(superstructure.elevator.setExtension(ElevatorConstants.AN)),
                    Commands.waitUntil(
                            () -> superstructure.elevator.isNearExtension(ElevatorConstants.AN))
                        .andThen(superstructure.gripper.setVoltage(GripperConstants.AN)))
                .onlyWhile(superstructure.gripper::getDetected)
                .andThen(superstructure.elevator.setExtension(ElevatorConstants.intake)));

    return routine.cmd();
  }

  public Command bargeToF() {
    final var routine = factory.newRoutine("barge to F");
    final var intake = routine.trajectory("bargetoF");
    routine.active().whileTrue(Commands.sequence(intake.resetOdometry(), intake.cmd()));
    routine
        .observe(intake.done())
        .onTrue(
            Commands.parallel(
                Commands.parallel(
                    superstructure.elevator.setExtension(ElevatorConstants.A3),
                    superstructure.gripper.setVoltage(GripperConstants.A23)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            Reef.centerFaces[AutoAlign.getBestFace(drive.getPose(), 0, 0)]
                                .rotateAround(
                                    Reef.centerFaces[AutoAlign.getBestFace(drive.getPose(), 0, 0)]
                                        .getTranslation(),
                                    Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command Ftobarge() {
    final var routine = factory.newRoutine("F to barge");
    final var score = routine.trajectory("Ftobarge");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));
    routine
        .observe(score.done())
        .onTrue(
            Commands.parallel(
                    Commands.waitUntil(this::atBarge)
                        .andThen(superstructure.elevator.setExtension(ElevatorConstants.AN)),
                    Commands.waitUntil(
                            () -> superstructure.elevator.isNearExtension(ElevatorConstants.AN))
                        .andThen(superstructure.gripper.setVoltage(GripperConstants.AN)))
                .onlyWhile(superstructure.gripper::getDetected)
                .andThen(superstructure.elevator.setExtension(ElevatorConstants.intake)));

    return routine.cmd();
  }

  public Command bargetoPLO() {
    final var routine = factory.newRoutine("barge to PLO");
    final var intake = routine.trajectory("bargetoPLO");

    routine
        .active()
        .whileTrue(
            Commands.parallel(
                Commands.sequence(intake.resetOdometry(), intake.cmd()),
                superstructure.elevator.setExtension(ElevatorConstants.intake)));

    routine
        .observe(intake.done())
        .onTrue(
            Commands.deadline(
                    Commands.waitUntil(superstructure.outtake::getDetected),
                    AutoAlign.translateToPose(
                        drive, () -> AutoAlign.getBestLoader(drive.getPose())))
                .andThen(
                    Commands.parallel(
                        superstructure.hopper.setVoltage(0),
                        superstructure.outtake.setVoltage(0))));
    return routine.cmd();
  }

  public Command LMtobarge() {
    return Commands.sequence(
        this.LMtoH().onlyWhile(superstructure.outtake::getDetected),
        Commands.waitSeconds(1),
        this.HtoI().onlyWhile(() -> !superstructure.gripper.getDetected()),
        Commands.waitSeconds(1),
        Commands.deadline(
            Commands.waitUntil(() -> DriverStation.getMatchTime() > 2.0),
            Commands.sequence(
                this.Itobarge().onlyWhile(superstructure.gripper::getDetected), this.bargetoPLO())),
        this.bargetoPLO());
  }

  public boolean atReef() {
    return drive
            .getPose()
            .getTranslation()
            .getDistance(AllianceFlipUtil.apply(FieldConstants.Reef.center))
        < ElevatorConstants.autoReefRaiseDistance;
  }

  public boolean atBranch() {
    return AutoAlign.isInTolerance(
        drive.getPose(),
        AutoAlign.getBestBranch(drive.getPose())
            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)));
  }

  public boolean atScore4() {
    return atBranch() && superstructure.elevator.isNearExtension(ElevatorConstants.L4);
  }

  public boolean atScore2() {
    return atBranch() && superstructure.elevator.isNearExtension(ElevatorConstants.L2);
  }

  public boolean atAlgae() {
    return AutoAlign.isInTolerance(
        drive.getPose(),
        AllianceFlipUtil.apply(Reef.algaeIntake[AutoAlign.getBestFace(drive.getPose(), 0, 0)]));
  }

  public boolean atBarge() {
    double threshold =
        DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red ? 10 : 7.5)
            .orElse(7.5);
    boolean reached = false;

    reached = (Math.abs(threshold - drive.getPose().getX()) < 0.1);
    Logger.recordOutput("AutoAlign/atBarge", reached);
    return reached;
  }

  public boolean readyToIntake() {
    return superstructure.elevator.getExtensionMeters() < 0.5;
  }
}
