package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;
import frc.robot.interfaces.motor.MotorIOSparkMax;
import frc.robot.util.subsystemUtils.ElevatorMechanism2d;

public class ElevatorIOSpark implements ElevatorIO {

  private final MotorIO motorLeft;
  private final MotorIO motorRight;
  private final ElevatorMechanism2d elevatorSIM =
      new ElevatorMechanism2d(
          "Elevator",
          ElevatorConstants.ELEVATOR_MAX_POSITION,
          ElevatorConstants.ELEVATOR_MIN_POSITION,
          Color.kSlateGray);

  public ElevatorIOSpark() {
    motorLeft =
        new MotorIOSparkMax(
            ElevatorConstants.MOTOR_LEFT_ID,
            MotorType.kBrushless,
            false,
            250,
            10.0,
            30,
            IdleMode.kBrake,
            1,
            1,
            false,
            -90,
            0,
            false);
    motorRight =
        new MotorIOSparkMax(
            ElevatorConstants.MOTOR_RIGHT_ID,
            MotorType.kBrushless,
            false,
            250,
            10.0,
            30,
            IdleMode.kBrake,
            1,
            1,
            false,
            -ElevatorConstants.ELEVATOR_MAX_POSITION,
            0,
            false);
    // reset();
    // elevatorPID = new PIDController(1, 0, 0);

    // motorLeft.setFollower(ElevatorConstants.MOTOR_RIGHT_ID);
    motorRight.configurePIDF(0.2, 0, 0, 0);
    motorLeft.configurePIDF(0.2, 0, 0, 0);
    // motorRight.setTypeEncoder(FeedbackSensor.kAlternateOrExternalEncoder, true);
    // motorRight.setOffset(motorRight.getMotorIOInputs().positionAlternateEncoder);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    MotorIOInputs motorIOInputs = motorLeft.getMotorIOInputs();
    inputs.appliedVoltsLeft = motorIOInputs.appliedVolts;
    inputs.currentAmpsLeft = motorIOInputs.currentAmps[0];
    inputs.velocityRadPerSecLeft = motorIOInputs.velocityRadPerSec;
    inputs.positionRotLeft = (motorIOInputs.positionRot);
    motorIOInputs = motorRight.getMotorIOInputs();
    inputs.appliedVoltsRight = motorIOInputs.appliedVolts;
    inputs.currentAmpsRight = motorIOInputs.currentAmps[0];
    inputs.velocityRadPerSecRight = motorIOInputs.velocityRadPerSec;
    inputs.positionRotRight = motorIOInputs.positionRot;
    inputs.positionAlternateEncoder = motorIOInputs.positionAlternateEncoder;
    elevatorSIM.update(inputs.positionRotLeft);
  }

  @Override
  public void setVoltage(double volts) {
    motorRight.setVoltage(volts);
    motorLeft.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    motorRight.setVelocity(velocityRadPerSec);
    motorLeft.setVelocity(velocityRadPerSec);
  }

  @Override
  public void set(double power) {
    motorRight.set(power);
    motorLeft.set(power);
  }

  @Override
  public void stop() {
    motorRight.stop();
    motorLeft.stop();
  }

  @Override
  public void runPosition(double position) {
    motorRight.setPosition(position);
    motorLeft.setPosition(position);
    elevatorSIM.setTargetPosition(position);
  }

  @Override
  public void reset() {
    motorRight.resetOffset();
    motorLeft.resetOffset();
  }
}
