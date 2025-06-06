package frc.robot.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IdleMode;
import frc.robot.subsystems.drive.Drive;

public class RobotModeTo extends InstantCommand {

  private final Drive drive;
  private IdleMode mode;

  public RobotModeTo(IdleMode idleMode, Drive drive) {
    this.mode = idleMode;
    this.drive = drive;
  }

  @Override
  public void initialize() {
    switch (mode) {
      case COAST:
        drive.setBrakeMode(false);
        break;
      default:
        drive.setBrakeMode(true);
        break;
    }
  }
}
