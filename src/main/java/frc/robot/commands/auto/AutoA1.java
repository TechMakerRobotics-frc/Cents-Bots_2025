// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.reef.DeliveryCoralSlow;
import frc.robot.commands.reef.L0;
import frc.robot.commands.reef.L1;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoA1 extends SequentialCommandGroup {
  /** Creates a new AutoA1. */
  public AutoA1(SwerveSubsystem swerve,
  Arm arm,
  Elevator elevator,
  Cradle cradle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new L1(arm, elevator, cradle),
                new DriveToX(swerve, 2).withTimeout(3),
                new DeliveryCoralSlow(cradle).withTimeout(3),
                new DriveToHeading(swerve, 90).withTimeout(2),
                new L0(arm, elevator, cradle),
                new DriveToHeading(swerve, 90).withTimeout(2)
                );
  }
}
