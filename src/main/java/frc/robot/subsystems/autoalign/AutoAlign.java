package frc.robot.subsystems.autoalign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.FieldConstants.Reef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoAlign {

  public static Pose2d getBestLoader(Pose2d currentPose) {
    Pose2d loader =
        AllianceFlipUtil.applyY(currentPose.getY()) > (FieldConstants.fieldWidth / 2)
            ? AllianceFlipUtil.apply(CoralStation.leftCenterFace)
            : AllianceFlipUtil.apply(CoralStation.rightCenterFace);

    Logger.recordOutput("AutoAlign/BestLoader", loader);
    Logger.recordOutput(
        "AutoAlign/BestLoaderDistance",
        loader.getTranslation().getDistance(currentPose.getTranslation()));

    return loader;
  }

  public static int getBestFace(Pose2d currentPose, double x, double y) {
    int bestIndex = 0;
    double bestScore = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 6; i++) {
      Translation2d faceLocation = AllianceFlipUtil.apply(Reef.centerFaces[i].getTranslation());

      Translation2d robotToFaceVector = faceLocation.minus(currentPose.getTranslation());
      Translation2d driverControlVector = new Translation2d(x, y);

      double faceDistanceScore = robotToFaceVector.getNorm();
      double driverInputScore;
      if (driverControlVector.getNorm() < .1) {
        driverInputScore = 0;
      } else {
        Rotation2d robotToFaceAngle = robotToFaceVector.getAngle();
        Rotation2d driverControlAngle = driverControlVector.getAngle();

        driverInputScore = driverControlAngle.minus(robotToFaceAngle).getCos() * 0; // DISABLED
      }

      Logger.recordOutput("AutoAlign/Reef/Face " + i + "/Distance", faceDistanceScore);
      Logger.recordOutput("AutoAlign/Reef/Face " + i + "/Input", driverInputScore);
      double faceScore = faceDistanceScore - driverInputScore;
      Logger.recordOutput("AutoAlign/Reef/Face " + i + "/Score", faceScore);

      if (faceScore < bestScore) {
        bestIndex = i;
        bestScore = faceScore;
      }
    }
    Pose2d bestFace = AllianceFlipUtil.apply(Reef.centerFaces[bestIndex]);
    Logger.recordOutput("AutoAlign/BestReef", bestFace);
    Logger.recordOutput(
        "AutoAlign/BestReefDistance",
        bestFace.getTranslation().getDistance(currentPose.getTranslation()));

    // return bestFace;
    return bestIndex;
  }

  public static Pose2d getBestBranch(Pose2d currentPose) {
    int bestFace = getBestFace(currentPose, 0, 0);
    Pose2d nearest =
        currentPose.nearest(
            Arrays.asList(
                AllianceFlipUtil.apply(Reef.branchesLeft[bestFace]),
                AllianceFlipUtil.apply(Reef.branchesRight[bestFace])));
    Logger.recordOutput("AutoAlign/BestBranch", nearest);
    return nearest;
  }

  public static Command translateToPose(Drive drive, Supplier<Pose2d> target) {
    return translateToPose(drive, target, () -> new ChassisSpeeds());
  }

  public static Command autoAimWithIntermediatePose(
      Drive drive, Supplier<Pose2d> intermediate, Supplier<Pose2d> end) {
    return translateToPose(drive, intermediate)
        .until(() -> isInTolerance(drive.getPose(), intermediate.get()))
        .andThen(translateToPose(drive, end));
  }

  /** Transforms the end pose by translationToIntermediate to get the intermediate pose */
  public static Command autoAimWithIntermediatePose(
      Drive drive, Supplier<Pose2d> end, Transform2d translationToIntermediate) {
    return autoAimWithIntermediatePose(
        drive, () -> end.get().transformBy(translationToIntermediate), end);
  }

  public static Command autoAimWithIntermediatePose(
      Drive drive, Supplier<Pose2d> intermediate, Supplier<Pose2d> end, Constraints constraints) {
    return translateToPose(drive, intermediate, ChassisSpeeds::new)
        .until(() -> isInTolerance(drive.getPose(), intermediate.get()))
        .andThen(translateToPose(drive, end, ChassisSpeeds::new));
  }

  /** Transforms the end pose by translationToIntermediate to get the intermediate pose */
  public static Command autoAimWithIntermediatePose(
      Drive drive,
      Supplier<Pose2d> end,
      Transform2d translationToIntermediate,
      Constraints constraints) {
    return autoAimWithIntermediatePose(
        drive, () -> end.get().transformBy(translationToIntermediate), end, constraints);
  }

  public static Command translateToPose(
      Drive drive, Supplier<Pose2d> target, Supplier<ChassisSpeeds> speedsModifier) {
    // This feels like a horrible way of getting around lambda final requirements
    // Is there a cleaner way of doing this?
    Logger.recordOutput("AutoAlign/Raw Target", target.get());

    final Pose2d cachedTarget[] = {new Pose2d()};
    final ProfiledPIDController headingController =
        // assume we can accelerate to max in 2/3 of a second
        new ProfiledPIDController(
            10.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                AutoAlignConstants.maxLinearSpeed, AutoAlignConstants.maxLinearAccel));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    final ProfiledPIDController vxController =
        new ProfiledPIDController(
            6.0,
            0.01,
            0.02,
            new TrapezoidProfile.Constraints(
                AutoAlignConstants.maxLinearSpeed, AutoAlignConstants.maxLinearAccel));
    final ProfiledPIDController vyController =
        new ProfiledPIDController(
            6.0,
            0.01,
            0.02,
            new TrapezoidProfile.Constraints(
                AutoAlignConstants.maxLinearSpeed, AutoAlignConstants.maxLinearAccel));
    return Commands.runOnce(
            () -> {
              cachedTarget[0] = target.get();
              final Transform2d diff = drive.getPose().minus(cachedTarget[0]);
              if (Constants.currentMode == Mode.SIM)
                Logger.recordOutput("AutoAlign/Cached Target", cachedTarget[0]);
              headingController.reset(
                  drive.getPose().getRotation().getRadians(),
                  drive.getVelocityFieldRelative().omegaRadiansPerSecond);
              vxController.reset(
                  drive.getPose().getX(), drive.getVelocityFieldRelative().vxMetersPerSecond);
              vyController.reset(
                  drive.getPose().getY(), drive.getVelocityFieldRelative().vyMetersPerSecond);
            })
        .andThen(
            drive.driveVelocityFieldRelative(
                () -> {
                  final Transform2d diff = drive.getPose().minus(cachedTarget[0]);
                  final ChassisSpeeds speeds =
                      MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(0.75)) // .25
                              && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(0.75)) // .25
                              && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 0.5)
                          ? new ChassisSpeeds().plus(speedsModifier.get())
                          : new ChassisSpeeds(
                                  vxController.calculate(
                                          drive.getPose().getX(), cachedTarget[0].getX())
                                      + vxController.getSetpoint().velocity,
                                  vyController.calculate(
                                          drive.getPose().getY(), cachedTarget[0].getY())
                                      + vyController.getSetpoint().velocity,
                                  headingController.calculate(
                                          drive.getPose().getRotation().getRadians(),
                                          cachedTarget[0].getRotation().getRadians())
                                      + headingController.getSetpoint().velocity)
                              .plus(speedsModifier.get());
                  // if (Constants.currentMode == Mode.SIM)
                  Logger.recordOutput(
                      "AutoAlign/Target Pose",
                      new Pose2d(
                          vxController.getSetpoint().position,
                          vyController.getSetpoint().position,
                          Rotation2d.fromRadians(headingController.getSetpoint().position)));
                  // if (Constants.currentMode == Mode.SIM)
                  Logger.recordOutput("AutoAlign/Target Speeds", speeds);
                  return speeds;
                }));
  }

  public static Command translateToXCoord(
      Drive drive, DoubleSupplier x, DoubleSupplier yVel, Supplier<Rotation2d> headingTarget) {

    final Pose2d cachedTarget[] = {new Pose2d()};
    final ProfiledPIDController headingController =
        // assume we can accelerate to max in 2/3 of a second
        new ProfiledPIDController(
            6.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                AutoAlignConstants.maxAngularSpeed, AutoAlignConstants.maxAngularAccel));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    final ProfiledPIDController vxController =
        new ProfiledPIDController(
            6.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                AutoAlignConstants.maxLinearSpeed, AutoAlignConstants.maxLinearAccel));
    return Commands.runOnce(
            () -> {
              cachedTarget[0] = new Pose2d(x.getAsDouble(), 0, headingTarget.get());
              if (Constants.currentMode == Mode.SIM)
                Logger.recordOutput("AutoAlign/Cached Target", cachedTarget[0]);
              headingController.reset(drive.getPose().getRotation().getRadians(), 0.0);
              vxController.reset(drive.getPose().getX(), 0.0);
            })
        .andThen(
            drive.driveVelocityFieldRelative(
                () -> {
                  final Transform2d diff = drive.getPose().minus(cachedTarget[0]);
                  final ChassisSpeeds speeds =
                      MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(0.75)) // .25
                              && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(0.75)) // .25
                              && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 0.5)
                          ? new ChassisSpeeds()
                          : new ChassisSpeeds(
                              vxController.calculate(drive.getPose().getX(), cachedTarget[0].getX())
                                  + vxController.getSetpoint().velocity,
                              // Use the inputted y velocity target
                              yVel.getAsDouble(),
                              headingController.calculate(
                                      drive.getPose().getRotation().getRadians(),
                                      cachedTarget[0].getRotation().getRadians())
                                  + headingController.getSetpoint().velocity);
                  if (Constants.currentMode == Mode.SIM)
                    Logger.recordOutput(
                        "AutoAlign/Target Pose",
                        new Pose2d(
                            vxController.getSetpoint().position,
                            0,
                            Rotation2d.fromRadians(headingController.getSetpoint().position)));
                  if (Constants.currentMode == Mode.SIM)
                    Logger.recordOutput("AutoAlign/Target Speeds", speeds);
                  return speeds;
                }));
  }

  public static boolean isInTolerance(Pose2d pose, Pose2d pose2) {
    final Transform2d diff = pose.minus(pose2);
    boolean threshold =
        MathUtil.isNear(
                0.0, Math.hypot(diff.getX(), diff.getY()), AutoAlignConstants.linearTolerance)
            && MathUtil.isNear(
                0.0, diff.getRotation().getRadians(), AutoAlignConstants.angularTolerance);
    Logger.recordOutput("AutoAlign/isInTolerance", threshold);
    return threshold;
  }

  public static boolean isInTolerance(Pose2d pose1, Pose2d pose2, ChassisSpeeds speeds) {
    return isInTolerance(
        pose1,
        pose2,
        speeds,
        AutoAlignConstants.linearTolerance,
        AutoAlignConstants.angularTolerance);
  }

  public static boolean isInTolerance(
      Pose2d pose1,
      Pose2d pose2,
      ChassisSpeeds speeds,
      double translationTolerance,
      double rotationTolerance) {
    final Transform2d diff = pose1.minus(pose2);
    return MathUtil.isNear(0.0, Math.hypot(diff.getX(), diff.getY()), translationTolerance)
        && MathUtil.isNear(0.0, diff.getRotation().getRadians(), rotationTolerance)
        && MathUtil.isNear(
            0,
            Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            AutoAlignConstants.velocityTolerance)
        && MathUtil.isNear(0.0, speeds.omegaRadiansPerSecond, 3.0);
  }

  public static boolean isInToleranceCoral(Pose2d pose, double x, double y) {
    final Transform2d diff = pose.minus(Reef.centerFaces[getBestFace(pose, x, y)]);
    return MathUtil.isNear(
            0.0, Math.hypot(diff.getX(), diff.getY()), AutoAlignConstants.linearTolerance)
        && MathUtil.isNear(
            0.0, diff.getRotation().getRadians(), AutoAlignConstants.angularTolerance);
  }

  public enum IntakeLocation {
    REEF,
    SOURCE
  }

  public static IntakeLocation closerIntake(Pose2d pose, double x, double y) {
    Pose2d nearestReef = AllianceFlipUtil.apply(Reef.centerFaces[getBestFace(pose, x, y)]);
    Pose2d nearestSource = getBestLoader(pose);

    double distanceToReef = pose.getTranslation().getDistance(nearestReef.getTranslation());
    double distanceToSource = pose.getTranslation().getDistance(nearestSource.getTranslation());
    IntakeLocation closest =
        distanceToReef <= distanceToSource ? IntakeLocation.REEF : IntakeLocation.SOURCE;
    Logger.recordOutput("AutoAlign/CloserIntake", closest);
    return closest;
  }
}
