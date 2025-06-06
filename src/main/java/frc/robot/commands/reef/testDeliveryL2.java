// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.reef;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class testDeliveryL2 extends SequentialCommandGroup {
  /** Creates a new testDelivery. */
  public testDeliveryL2(Arm arm, Elevator elevator, Intake intake, Cradle cradle, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(arm, elevator, intake, cradle, drive);

    addCommands( // new ParallelCommandGroup(
        //  new DriveTo(drive,2.5,2.5, 10),
        new L2(arm, elevator, cradle)
        // ),
        ,
        new DeliveryCoral(cradle),
        // new DriveTo(drive, 2,2,10),
        new L0(arm, elevator, cradle));
  }
}
