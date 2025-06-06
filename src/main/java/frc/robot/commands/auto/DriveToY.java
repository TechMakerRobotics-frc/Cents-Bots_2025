package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToY extends Command {
  private final SwerveSubsystem swerve;
  private double distanceInMeters;
  private Pose2d startPose;
  private PIDController controller;

  public DriveToY(SwerveSubsystem swerve, double distanceInMeters) {
    this.distanceInMeters = distanceInMeters;
    this.swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new PIDController(3, 1, 0.1);
    controller.setSetpoint(Math.abs(distanceInMeters));
    startPose = swerve.getPose();
    controller.setTolerance(Math.abs(distanceInMeters) * 0.02);
    controller.setIZone(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedInMetersPerSecond;
    if (distanceInMeters > 0) {
      speedInMetersPerSecond =
          controller.calculate(
              startPose.getTranslation().getDistance(swerve.getPose().getTranslation()));
      swerve.drive(new ChassisSpeeds(0, speedInMetersPerSecond, 0));
    } else {
      speedInMetersPerSecond =
          controller.calculate(
              swerve.getPose().getTranslation().getDistance(startPose.getTranslation()));
      swerve.drive(new ChassisSpeeds(0, -speedInMetersPerSecond, 0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
