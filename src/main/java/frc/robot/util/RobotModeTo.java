package frc.robot.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IdleMode;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotModeTo extends InstantCommand {

  private final SwerveSubsystem drive;
  private IdleMode mode;

  public RobotModeTo(IdleMode idleMode, SwerveSubsystem drive) {
    this.mode = idleMode;
    this.drive = drive;
  }

  @Override
  public void initialize() {
    switch (mode) {
      case COAST:
        drive.setMotorBrake(false);
        break;
      default:
        drive.setMotorBrake(true);
        break;
    }
  }
}
