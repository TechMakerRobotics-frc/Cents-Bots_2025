// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.reef;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRightReef extends Command {
  private SwerveSubsystem swerve;
  private VisionIO io;
  private PIDController xController, yController, rController;
  private Timer timer = new Timer();

  public AlignRightReef(SwerveSubsystem swerve, Vision vision) {
    this.swerve = swerve;
    io = vision.getVisionIO(VisionConstants.L_CAM_NAME);
    addRequirements(swerve, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController =
        new PIDController(
            CommandConstantsReef.kP_x, CommandConstantsReef.kI_x, CommandConstantsReef.kD_x);

    yController =
        new PIDController(
            CommandConstantsReef.kP_y, CommandConstantsReef.kI_y, CommandConstantsReef.kD_y);
    rController =
        new PIDController(
            CommandConstantsReef.kP_r, CommandConstantsReef.kI_r, CommandConstantsReef.kD_r);
    xController.setIZone(0.25);
    yController.setIZone(0.25);
    rController.setIZone(5);
    rController.enableContinuousInput(-180, 180);
    xController.setSetpoint(CommandConstantsReef.RIGHTX);
    yController.setSetpoint(CommandConstantsReef.RIGHTY);
    rController.setSetpoint(CommandConstantsReef.RIGHTR);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = 0, y = 0, r = 0;
    if (io.getId() > 0) {
      x = xController.calculate(io.getX());
      y = yController.calculate(io.getY());
      r = rController.calculate(io.getR());
      timer.restart();
    }
    swerve.drive(new Translation2d(-x, -y), -r, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.5;
  }
}
