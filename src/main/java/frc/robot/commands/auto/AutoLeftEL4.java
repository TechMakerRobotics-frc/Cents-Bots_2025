package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.reef.AlignLeftReef;
import frc.robot.commands.reef.DeliveryCoral;
import frc.robot.commands.reef.IntakeCoral;
import frc.robot.commands.reef.L0;
import frc.robot.commands.reef.L4;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutoLeftEL4 extends SequentialCommandGroup {
  public AutoLeftEL4(
      SwerveSubsystem swerve,
      Vision vision,
      Arm arm,
      Elevator elevator,
      Cradle cradle,
      Funnel funnel) {
    addCommands(
        new DriveToX(swerve, 2).withTimeout(2),
        new DriveToHeading(swerve, 60).withTimeout(2),
        new ParallelCommandGroup(
            new IntakeCoral(cradle, funnel), new DriveUntilTag(swerve, vision).withTimeout(5)),
        new ParallelCommandGroup(new AlignLeftReef(swerve, vision), new L4(arm, elevator, cradle))
            .withTimeout(3),
        new DriveToX(swerve, 0.1).withTimeout(1),
        new DeliveryCoral(cradle).withTimeout(2),
        new DriveToX(swerve, -0.5).withTimeout(2),
        new L0(arm, elevator, cradle).withTimeout(2));
  }
}
