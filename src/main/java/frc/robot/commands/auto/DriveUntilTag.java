package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;

public class DriveUntilTag extends Command {
  private SwerveSubsystem swerve;
  private VisionIO ioR, ioL;

  public DriveUntilTag(SwerveSubsystem swerve, Vision vision) {
    this.swerve = swerve;
    ioR = vision.getVisionIO(VisionConstants.R_CAM_NAME);
    ioL = vision.getVisionIO(VisionConstants.L_CAM_NAME);
    addRequirements(swerve, vision);
  }

  @Override
  public void initialize() {
    swerve.drive(new Translation2d(.5, 0), 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(new Translation2d(.5, 0), 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int idL = ioL.getId();
    int idR = ioR.getId();
    if (idL > 0 && idR > 0) return true;
    else return false;
  }
}
