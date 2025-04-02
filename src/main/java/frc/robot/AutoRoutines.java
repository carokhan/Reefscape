package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.autoalign.AutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorConstants;
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
    final var start = routine.trajectory("LMtoH");
    routine.active().whileTrue(Commands.sequence(start.resetOdometry(), start.cmd()));

    routine
        .observe(start.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore)
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
    final var start = routine.trajectory("ROtoE");
    routine.active().whileTrue(Commands.sequence(start.resetOdometry(), start.cmd()));

    routine
        .observe(start.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore)
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
    final var start = routine.trajectory("LOtoI");
    routine.active().whileTrue(Commands.sequence(start.resetOdometry(), start.cmd()));

    routine
        .observe(start.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore)
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
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
            Commands.parallel(
                    AutoAlign.translateToPose(
                        drive, () -> AutoAlign.getBestLoader(drive.getPose())),
                    Commands.waitUntil(superstructure.outtake::getDetected))
                .andThen(
                    Commands.parallel(
                        superstructure.hopper.setVoltage(0),
                        superstructure.outtake.setVoltage(0))));
    return routine.cmd();
  }

  public Command PLOtoK() {
    final var routine = factory.newRoutine("PLO to K");
    final var score2 = routine.trajectory("PLOtoK");

    routine.active().whileTrue(Commands.sequence(score2.resetOdometry(), score2.cmd()));

    routine
        .observe(score2.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore)
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public Command LOtoItoPLOtoK() {
    return Commands.sequence(
        this.LOtoI().onlyWhile(superstructure.outtake::getDetected),
        this.ItoPLO().onlyWhile(() -> !superstructure.outtake.getDetected()),
        this.PLOtoK());

    // superstructure.outtake.setSimDetected(true);
    // final var routine = factory.newRoutine("LO to I to PLO");
    // final var score1 = routine.trajectory("LOtoI");
    // final var intake = routine.trajectory("ItoPLO");
    // final var score2 = routine.trajectory("PLOtoK");
    // routine.active().whileTrue(Commands.sequence(score1.resetOdometry(),
    // score1.cmd()));

    // routine
    // .observe(score1.done())
    // .onTrue(
    // Commands.parallel(
    // Commands.waitUntil(this::atReef)
    // .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
    // Commands.waitUntil(this::atScore)
    // .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
    // AutoAlign.translateToPose(
    // drive,
    // () ->
    // AutoAlign.getBestBranch(drive.getPose())
    // .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg))))
    // .onlyWhile(superstructure.outtake::getDetected)
    // .andThen(
    // Commands.parallel(
    // superstructure.elevator.setExtension(ElevatorConstants.intake),
    // superstructure.hopper.setVoltage(HopperConstants.intake),
    // superstructure.outtake.index())));
    // routine.observe(this::readyToIntake).onTrue(intake.cmd());
    // routine
    // .observe(intake.done())
    // .onTrue(
    // Commands.parallel(
    // AutoAlign.translateToPose(
    // drive, () -> AutoAlign.getBestLoader(drive.getPose())),
    // Commands.waitUntil(superstructure.outtake::getDetected))
    // .andThen(
    // Commands.parallel(
    // superstructure.hopper.setVoltage(0),
    // superstructure.outtake.setVoltage(0))));
    // routine.observe(superstructure.outtake::getDetected).onTrue(score2.cmd());
    // routine
    // .observe(score2.done())
    // .onTrue(
    // Commands.parallel(
    // Commands.waitUntil(this::atReef)
    // .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
    // Commands.waitUntil(this::atScore)
    // .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
    // AutoAlign.translateToPose(
    // drive,
    // () ->
    // AutoAlign.getBestBranch(drive.getPose())
    // .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));

    // return routine.cmd();
  }

  public Command LOtoJ() {
    superstructure.outtake.setSimDetected(true);
    final var routine = factory.newRoutine("LO to J");
    final var start = routine.trajectory("LOtoJ");
    routine.active().whileTrue(Commands.sequence(start.resetOdometry(), start.cmd()));
    routine
        .observe(start.done())
        .onTrue(
            Commands.parallel(
                Commands.waitUntil(this::atReef)
                    .andThen(superstructure.elevator.setExtension(ElevatorConstants.L4)),
                Commands.waitUntil(this::atScore)
                    .andThen(superstructure.outtake.setVoltage(OuttakeConstants.L4)),
                AutoAlign.translateToPose(
                    drive,
                    () ->
                        AutoAlign.getBestBranch(drive.getPose())
                            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)))));
    return routine.cmd();
  }

  public boolean atReef() {
    return drive
            .getPose()
            .getTranslation()
            .getDistance(AllianceFlipUtil.apply(FieldConstants.Reef.center))
        < ElevatorConstants.reefRaiseDistance;
  }

  public boolean atBranch() {
    return AutoAlign.isInTolerance(
        drive.getPose(),
        AutoAlign.getBestBranch(drive.getPose())
            .plus(new Transform2d(new Translation2d(), Rotation2d.k180deg)));
  }

  public boolean atScore() {
    return atBranch() && superstructure.elevator.isNearExtension(ElevatorConstants.L4);
  }

  public boolean readyToIntake() {
    return superstructure.elevator.getExtensionMeters() < 0.5;
  }

  // public void runPath(
  // AutoRoutine routine,
  // String startPos,
  // String endPos,
  // String nextPos,
  // HashMap<String, AutoTrajectory> steps) {
  // routine
  // .observe(
  // steps
  // .get(startPos + "to" + endPos)
  // .atTime(
  // steps.get(startPos + "to" + endPos).getRawTrajectory().getTotalTime()
  // - (endPos.length() == 1 ? 0.5 : 0.0)))
  // .onTrue(
  // Commands.sequence(
  // endPos.length() == 3
  // ? intakeInAuto(() -> steps.get(startPos + "to" + endPos).getFinalPose())
  // : Commands.sequence(
  // endPos.length() == 1
  // ? scoreInAuto(
  // () -> steps.get(startPos + "to" + endPos).getFinalPose().get())
  // : AutoAlign.translateToPose(
  // drive,
  // () -> steps.get(startPos + "to" + endPos).getFinalPose().get())
  // .until(
  // () ->
  // AutoAlign.isInTolerance(
  // drive.getPose(),
  // steps
  // .get(startPos + "to" + endPos)
  // .getFinalPose()
  // .get()))
  // .withTimeout(2.0)),
  // steps.get(endPos + "to" + nextPos).cmd()));
  // }
}
