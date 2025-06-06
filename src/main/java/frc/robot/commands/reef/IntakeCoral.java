// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.reef;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.cradle.CradleConstants;
import frc.robot.subsystems.funnel.Funnel;

public class IntakeCoral extends Command {
  /** Creates a new deliveryCradle. */
  private final Cradle cradle;

  private final Funnel funnel;

  enum State {
    intakeing,
    backing,
    forward,
    adjusting,
    adjusted
  }

  private State state = State.intakeing;

  public IntakeCoral(Cradle cradle, Funnel funnel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cradle = cradle;
    this.funnel = funnel;
    addRequirements(cradle, funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cradle.set(0.75);
    funnel.setConveyor(0.3);
    state = State.intakeing;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cradle.stop();
    funnel.setConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cradle.frontSensorIsTrue();
  }
}
