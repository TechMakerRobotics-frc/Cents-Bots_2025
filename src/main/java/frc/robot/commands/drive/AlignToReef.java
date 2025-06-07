package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.AlignConstants;
import frc.robot.commands.CommandConstants.MoveHConstants;
import frc.robot.commands.CommandConstants.MoveXConstants;
import frc.robot.commands.CommandConstants.MoveYConstants;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToReef extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = -1;

  public AlignToReef(boolean isRightScore, SwerveSubsystem drivebase) {
    xController = new PIDController(MoveXConstants.k_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(MoveYConstants.k_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(MoveHConstants.k_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(AlignConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(AlignConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(AlignConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(AlignConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? AlignConstants.Y_SETPOINT_REEF_ALIGNMENT : -AlignConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(AlignConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-left");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-left") && LimelightHelpers.getFiducialID("limelight-left") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
      SmartDashboard.putNumber("x", positions[2]);

      double xSpeed = xController.calculate(positions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(AlignConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(AlignConstants.POSE_VALIDATION_TIME);
  }
}