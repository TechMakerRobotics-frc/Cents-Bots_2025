package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToHeading extends Command {
  private final SwerveSubsystem swerve;
  private double AngleInDegrees;
  private PIDController controller;

  public DriveToHeading(SwerveSubsystem swerve, double AngleInDegrees) {
    this.AngleInDegrees = AngleInDegrees;
    this.swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new PIDController(0.1, 0.01, 0.0);
    controller.setSetpoint(swerve.getHeading().getDegrees() - AngleInDegrees);
    controller.setTolerance(Math.abs(AngleInDegrees) * 0.02);
    controller.setIZone(5);
    controller.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedInMetersPerSecond;
    speedInMetersPerSecond = controller.calculate(swerve.getHeading().getDegrees());
    swerve.drive(new ChassisSpeeds(0, 0, speedInMetersPerSecond));
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
