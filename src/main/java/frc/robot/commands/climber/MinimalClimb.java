package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Climber;

public class MinimalClimb extends InstantCommand {
  private final Climber climber;

  public MinimalClimb(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setMinimalPowerClimb();
  }
}
