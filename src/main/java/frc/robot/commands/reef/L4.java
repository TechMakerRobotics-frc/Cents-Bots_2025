// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.reef;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.cradle.CradleConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class L4 extends Command {

  private final Arm arm;
  private final Cradle cradle;
  private final Elevator elevator;
  private boolean elevatorHasCommand = false;

  public L4(Arm arm, Elevator elevator, Cradle cradle) {
    this.arm = arm;
    this.cradle = cradle;
    this.elevator = elevator;

    addRequirements(arm, elevator, cradle);
  }

  @Override
  public void initialize() {

    if (cradle.frontSensorIsTrue()) {
      arm.runPosition(ArmConstants.ARM_POSITION_L4);
      elevatorHasCommand = false;
    }
  }

  @Override
  public void execute() {
    if (!elevatorHasCommand && arm.getPosition() > 0.2) {
      elevatorHasCommand = true;
      cradle.runPosition(CradleConstants.CRADLE_POSITION_L4);
      elevator.runPosition(ElevatorConstants.ELEVATOR_MAX_POSITION);
    }
    if (cradle.getEncoder() > CradleConstants.OUT_POSITION) {
      cradle.set(0);
    } else if (arm.getPosition() > 0.1) {
      cradle.set(0.2);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (cradle.atSetpoint()
            && arm.atSetpoint()
            && elevator.atSetpoint()
            && elevatorHasCommand
            && cradle.getEncoder() > CradleConstants.OUT_POSITION)
        || !cradle.frontSensorIsTrue();
  }
}
