package frc.robot.subsystems.funnel;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;
import frc.robot.interfaces.motor.MotorIOSparkMax;

public class FunnelIOSpark implements FunnelIO {

  private final MotorIO intakeMotor;
  private final MotorIO conveyorMotor;

  public FunnelIOSpark() {
    intakeMotor =
        new MotorIOSparkMax(
            FunnelConstants.INTAKE_MOTOR_ID,
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
    conveyorMotor =
        new MotorIOSparkMax(
            FunnelConstants.CONVEYOR_MOTOR_ID,
            MotorType.kBrushless,
            false,
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
  public void updateInputs(FunnelIOInputs inputs) {
    MotorIOInputs motorIOInputs = intakeMotor.getMotorIOInputs();
    inputs.appliedVolts = motorIOInputs.appliedVolts;
    inputs.currentAmps = motorIOInputs.currentAmps[0];
    inputs.velocityRadPerSec = motorIOInputs.velocityRadPerSec;
    inputs.positionRad = Units.rotationsToRadians(motorIOInputs.positionRot);

    motorIOInputs = conveyorMotor.getMotorIOInputs();
    inputs.appliedVolts = motorIOInputs.appliedVolts;
    inputs.currentAmps = motorIOInputs.currentAmps[0];
    inputs.velocityRadPerSec = motorIOInputs.velocityRadPerSec;
    inputs.positionRad = Units.rotationsToRadians(motorIOInputs.positionRot);
  }

  @Override
  public void setVoltageIntakeMotor(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setVoltageConveyorMotor(double volts) {
    conveyorMotor.setVoltage(volts);
  }

  @Override
  public void setVelocityIntakeMotor(double velocityRadPerSec) {
    intakeMotor.setVelocity(velocityRadPerSec);
  }

  @Override
  public void setVelocityConveyorMotor(double velocityRadPerSec) {
    conveyorMotor.setVelocity(velocityRadPerSec);
  }

  @Override
  public void stopAll() {
    intakeMotor.stop();
    conveyorMotor.stop();
  }

  @Override
  public void stopIntakeMotor() {
    intakeMotor.stop();
  }

  @Override
  public void stopConveyorMotor() {
    conveyorMotor.stop();
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
  public void setConveyor(double power) {
    conveyorMotor.set(power);
  }
}
