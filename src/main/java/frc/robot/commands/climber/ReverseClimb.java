package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Climber;

public class ReverseClimb extends InstantCommand {
  private final Climber climber;

  public ReverseClimb(Climber climber) {
    this.climber = climber;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setReversePowerClimb();
  }
}
