package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.CommandConstants.MoveHConstants;
import frc.robot.commands.CommandConstants.MoveXConstants;
import frc.robot.commands.CommandConstants.MoveYConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class GoToFieldPose extends Command {
  private final SwerveSubsystem swerve;
  private final HolonomicDriveController controller;

  private final Pose2d targetPose;

  private final LoggedNetworkNumber kXP =
      new LoggedNetworkNumber("/Tuning/X/KP", MoveXConstants.k_P);
  private final LoggedNetworkNumber kXI =
      new LoggedNetworkNumber("/Tuning/X/KI", MoveXConstants.k_I);
  private final LoggedNetworkNumber kXD =
      new LoggedNetworkNumber("/Tuning/X/KD", MoveXConstants.k_D);

  private final LoggedNetworkNumber kYP =
      new LoggedNetworkNumber("/Tuning/Y/KP", MoveYConstants.k_P);
  private final LoggedNetworkNumber kYI =
      new LoggedNetworkNumber("/Tuning/Y/KI", MoveYConstants.k_I);
  private final LoggedNetworkNumber kYD =
      new LoggedNetworkNumber("/Tuning/Y/KD", MoveYConstants.k_D);

  private final LoggedNetworkNumber kHP =
      new LoggedNetworkNumber("/Tuning/H/KP", MoveHConstants.k_P);
  private final LoggedNetworkNumber kHI =
      new LoggedNetworkNumber("/Tuning/H/KI", MoveHConstants.k_I);
  private final LoggedNetworkNumber kHD =
      new LoggedNetworkNumber("/Tuning/H/KD", MoveHConstants.k_D);

  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;

  public GoToFieldPose(
      SwerveSubsystem swerve, double fieldX, double fieldY, double headingDegrees) {
    this.swerve = swerve;

    xController = new PIDController(kXP.get(), kXI.get(), kXD.get());
    yController = new PIDController(kYP.get(), kYI.get(), kYD.get());

    thetaController =
        new ProfiledPIDController(
            kHP.get(),
            kHI.get(),
            kHD.get(),
            new TrapezoidProfile.Constraints(Math.toRadians(360), Math.toRadians(720)));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller = new HolonomicDriveController(xController, yController, thetaController);
    controller.setTolerance(new Pose2d(0.01, 0.01, Rotation2d.fromDegrees(1.0)));

    this.targetPose = new Pose2d(fieldX, fieldY, Rotation2d.fromDegrees(headingDegrees));

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("/GoToFieldPose/X/Setpoint", targetPose.getX());
    Logger.recordOutput("/GoToFieldPose/Y/Setpoint", targetPose.getY());
    Logger.recordOutput("/GoToFieldPose/H/Setpoint", targetPose.getRotation().getDegrees());

    xController.setPID(kXP.get(), kXI.get(), kXD.get());
    yController.setPID(kYP.get(), kYI.get(), kYD.get());
    thetaController.setPID(kHP.get(), kHI.get(), kHD.get());
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();

    ChassisSpeeds targetSpeeds =
        controller.calculate(
            currentPose, targetPose, Constants.MAX_SPEED, targetPose.getRotation());

    swerve.drive(targetSpeeds);
    logValues(currentPose, targetSpeeds);
  }

  @Override
  public boolean isFinished() {
    return controller.atReference();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.lock();
  }

  private void logValues(Pose2d currentPose, ChassisSpeeds targetSpeeds) {
    Logger.recordOutput("/GoToFieldPose/X/CurrentPoseValue", currentPose.getX());
    Logger.recordOutput("/GoToFieldPose/Y/CurrentPoseValue", currentPose.getY());
    Logger.recordOutput(
        "/GoToFieldPose/H/CurrentPoseValue", currentPose.getRotation().getDegrees());

    Logger.recordOutput("/GoToFieldPose/X/Error", targetPose.getX() - currentPose.getX());
    Logger.recordOutput("/GoToFieldPose/Y/Error", targetPose.getY() - currentPose.getY());
    Logger.recordOutput(
        "/GoToFieldPose/H/Error",
        targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());

    Logger.recordOutput("/GoToFieldPose/X/Vx Setpoint", targetSpeeds.vxMetersPerSecond);
    Logger.recordOutput("/GoToFieldPose/Y/Vy Setpoint", targetSpeeds.vyMetersPerSecond);
    Logger.recordOutput("/GoToFieldPose/H/VOmega Setpoint", targetSpeeds.omegaRadiansPerSecond);

    Logger.recordOutput(
        "/GoToFieldPose/DistanceToTarget",
        currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    Logger.recordOutput("/GoToFieldPose/AtSetpoint", controller.atReference());
    Logger.recordOutput("/GoToFieldPose/Finished", isFinished());
  }
}
