package frc.robot.commands.reef;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.cradle.CradleConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeliveryCoral extends Command {
  /** Creates a new DeliveryCoral. */
  private final Cradle cradle;

  public DeliveryCoral(Cradle cradle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cradle = cradle;
    addRequirements(cradle);
  }

  @Override
  public void initialize() {
    cradle.set(CradleConstants.INTAKE_OUT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cradle.stop();
  }

  @Override
  public boolean isFinished() {
    return cradle.frontSensorIsTrue() == false;
  }
}
