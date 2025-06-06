package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;
import frc.robot.interfaces.motor.MotorIOSparkMax;

public class IntakeIOSpark implements IntakeIO {

  private final MotorIO intakeMotor;
  private final MotorIO rollerMotor;

  public IntakeIOSpark() {
    intakeMotor =
        new MotorIOSparkMax(
            IntakeConstants.INTAKE_MOTOR_ID,
            MotorType.kBrushless,
            true,
            250,
            10.0,
            30,
            IdleMode.kBrake,
            34,
            1,
            false,
            -1,
            1,
            false);
    rollerMotor =
        new MotorIOSparkMax(
            IntakeConstants.ROLLER_MOTOR_ID,
            MotorType.kBrushless,
            true,
            250,
            10.0,
            30,
            IdleMode.kBrake,
            7.0 / 150.0,
            1,
            false,
            -1,
            1,
            false);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputsIntake, IntakeIOInputs inputsRoller) {
    MotorIOInputs motorIOInputsIntake = intakeMotor.getMotorIOInputs();
    inputsIntake.appliedVolts = motorIOInputsIntake.appliedVolts;
    inputsIntake.currentAmps = motorIOInputsIntake.currentAmps[0];
    inputsIntake.velocityRadPerSec = motorIOInputsIntake.velocityRadPerSec;
    inputsIntake.positionRad = Units.rotationsToRadians(motorIOInputsIntake.positionRot);

    MotorIOInputs motorIOInputsRoller = rollerMotor.getMotorIOInputs();
    inputsRoller.appliedVolts = motorIOInputsRoller.appliedVolts;
    inputsRoller.currentAmps = motorIOInputsRoller.currentAmps[0];
    inputsRoller.velocityRadPerSec = motorIOInputsRoller.velocityRadPerSec;
    inputsRoller.positionRad = Units.rotationsToRadians(motorIOInputsRoller.positionRot);
  }

  @Override
  public void setVoltageIntakeMotor(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setVoltageRollerMotor(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void setVelocityIntakeMotor(double velocityRadPerSec) {
    intakeMotor.setVelocity(velocityRadPerSec);
  }

  @Override
  public void setVelocityRollerMotor(double velocityRadPerSec) {
    rollerMotor.setVelocity(velocityRadPerSec);
  }

  @Override
  public void stopAll() {
    intakeMotor.stop();
    rollerMotor.stop();
  }

  @Override
  public void stopIntakeMotor() {
    intakeMotor.stop();
  }

  @Override
  public void stopRollerMotor() {
    rollerMotor.stop();
  }

  @Override
  public void resetPosition() {
    intakeMotor.resetOffset();
  }

  @Override
  public void setIntake(double power) {
    intakeMotor.set(power);
  }

  @Override
  public void setRoller(double power) {
    rollerMotor.set(power);
  }

  @Override
  public void runPosition(double position) {
    intakeMotor.setPosition(position);
  }
}
