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

public class L2 extends Command {

  private final Arm arm;
  private final Cradle cradle;
  private boolean cradleCommand = false;

  public L2(Arm arm, Elevator elevator, Cradle cradle) {
    this.arm = arm;
    this.cradle = cradle;

    addRequirements(arm, elevator, cradle);
  }

  @Override
  public void initialize() {
    if (cradle.frontSensorIsTrue()) {
      cradleCommand = false;
      arm.runPosition(ArmConstants.ARM_POSITION_L2);
    }
  }

  @Override
  public void execute() {

    if (arm.getPosition() > 0.05 && !cradleCommand) {
      cradle.runPosition(CradleConstants.CRADLE_POSITION_L2);
      cradleCommand = true;
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
            && cradleCommand
            && cradle.getEncoder() > CradleConstants.OUT_POSITION)
        || !cradle.frontSensorIsTrue();
  }
}
