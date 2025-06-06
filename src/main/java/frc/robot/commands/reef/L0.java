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

public class L0 extends Command {

  private final Arm arm;
  private final Cradle cradle;
  private final Elevator elevator;
  private boolean armSettedZero = false;

  public L0(Arm arm, Elevator elevator, Cradle cradle) {
    this.arm = arm;
    this.cradle = cradle;
    this.elevator = elevator;

    addRequirements(arm, elevator, cradle);
  }

  @Override
  public void initialize() {
    cradle.runPosition(CradleConstants.CRADLE_POSITION_L1);
    arm.runPosition(ArmConstants.ARM_POSITION_L1);
    elevator.runPosition(ElevatorConstants.ELEVATOR_MIN_POSITION);
    armSettedZero = false;
  }

  @Override
  public void execute() {
   
    if (elevator.atZero() && !armSettedZero) {
      arm.runPosition(ArmConstants.ARM_MIN_POSITION);
      cradle.setZero();
      armSettedZero = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    cradle.setZero();
  }

  @Override
  public boolean isFinished() {
    return (arm.atZero() && armSettedZero);
  }
}
