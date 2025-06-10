package frc.robot.subsystems.cradle;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;
import frc.robot.interfaces.motor.MotorIOSparkMax;
import frc.robot.util.LoggedTunableNumber;

public class CradleIOSpark implements CradleIO {

  private final MotorIO motorLeft;
  private final MotorIO motorRight;
  private final MotorIO motorWrist;

  private final DigitalInput frontSensor;
  private final DigitalInput backSensor;
  private LoggedTunableNumber kp =
      new LoggedTunableNumber("/Cradle/P - Turn", CradleConstants.WRIST_kP);
  private LoggedTunableNumber ki =
      new LoggedTunableNumber("/Cradle/I - Turn", CradleConstants.WRIST_kI);
  private LoggedTunableNumber kd =
      new LoggedTunableNumber("/Cradle/D - Turn", CradleConstants.WRIST_kD);

  public CradleIOSpark() {
    motorLeft =
        new MotorIOSparkMax(
            CradleConstants.MOTOR_LEFT_ID,
            MotorType.kBrushless,
            false,
            250,
            10.0,
            30,
            IdleMode.kBrake,
            1,
            1,
            false,
            -0.5,
            0.5,
            false);
    motorRight =
        new MotorIOSparkMax(
            CradleConstants.MOTOR_RIGHT_ID,
            MotorType.kBrushless,
            true,
            250,
            10.0,
            30,
            IdleMode.kBrake,
            1,
            1,
            false,
            -0.5,
            0.5,
            false);
    motorWrist =
        new MotorIOSparkMax(
            CradleConstants.MOTOR_WRIST_ID,
            MotorType.kBrushless,
            false,
            250,
            10.0,
            80,
            IdleMode.kBrake,
            0.125,
            1,
            false,
            -1,
            1,
            false);
    // motorWrist.setAbsoluteEncoderPosition();
    motorWrist.configurePID(
        CradleConstants.WRIST_kP, CradleConstants.WRIST_kI, CradleConstants.WRIST_kD);
    Timer.delay(2);
    motorWrist.setOffset(0);
    // setZero();

    frontSensor = new DigitalInput(CradleConstants.FRONT_SENSOR_PORT);
    backSensor = new DigitalInput(CradleConstants.BACK_SENSOR_PORT);
  }

  @Override
  public void setZero() {
    motorWrist.setOffset(motorWrist.getAbsEncoderPosition() - CradleConstants.OFFSET);
    motorWrist.setPosition(0);
  }

  @Override
  public void resetEncoder() {
    motorLeft.resetOffset();
    motorRight.resetOffset();
  }

  @Override
  public double getEncoder() {
    return (motorLeft.getMotorIOInputs().positionRot + motorRight.getMotorIOInputs().positionRot)
        / 2.0;
  }

  @Override
  public void updateInputs(
      CradleIOInputs motorLeftInputs,
      CradleIOInputs motorRightInputs,
      CradleIOInputs motorWristInputs) {
    MotorIOInputs leftMotorIOInputs = motorLeft.getMotorIOInputs();
    motorLeftInputs.appliedVolts = leftMotorIOInputs.appliedVolts;
    motorLeftInputs.currentAmps = leftMotorIOInputs.currentAmps[0];
    motorLeftInputs.velocityRadPerSec = leftMotorIOInputs.velocityRadPerSec;
    motorLeftInputs.positionRad = Units.rotationsToRadians(leftMotorIOInputs.positionRot);
    motorLeftInputs.positionRot = leftMotorIOInputs.positionRot;

    MotorIOInputs rightMotorIOInputs = motorRight.getMotorIOInputs();
    motorRightInputs.appliedVolts = rightMotorIOInputs.appliedVolts;
    motorRightInputs.currentAmps = rightMotorIOInputs.currentAmps[0];
    motorRightInputs.velocityRadPerSec = rightMotorIOInputs.velocityRadPerSec;
    motorRightInputs.positionRad = Units.rotationsToRadians(rightMotorIOInputs.positionRot);
    motorRightInputs.positionRot = rightMotorIOInputs.positionRot;

    MotorIOInputs wristMotorIOInputs = motorWrist.getMotorIOInputs();
    motorWristInputs.appliedVolts = wristMotorIOInputs.appliedVolts;
    motorWristInputs.currentAmps = wristMotorIOInputs.currentAmps[0];
    motorWristInputs.velocityRadPerSec = wristMotorIOInputs.velocityRadPerSec;
    motorWristInputs.positionRad = Units.rotationsToRadians(wristMotorIOInputs.positionRot);
    motorWristInputs.positionRot = wristMotorIOInputs.positionRot;
    motorWristInputs.AbsolutePosition = motorWrist.getAbsEncoderPosition();
    if (kp.hasChanged(hashCode()) || ki.hasChanged(hashCode()) || kd.hasChanged(hashCode())) {
      motorWrist.configurePID(kp.get(), ki.get(), kd.get());
    }
  }

  @Override
  public void setVoltage(double volts) {
    motorLeft.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    motorLeft.setVelocity(velocityRadPerSec);
  }

  @Override
  public void stop() {
    motorLeft.stop();
    motorRight.stop();
  }

  @Override
  public void stopMoviment() {
    motorWrist.stop();
  }

  @Override
  public void set(double power) {
    if (power > 0.7) power = 0.7;
    motorRight.set(power);
    motorLeft.set(power);
  }

  @Override
  public void setMoviment(double power) {
    motorWrist.set(power);
  }

  @Override
  public void setPosition(double position) {
    motorWrist.setPosition(position);
  }

  @Override
  public void setInvertPower(double power) {
    motorLeft.set(power);
    motorRight.set(-power);
  }

  @Override
  public boolean frontSensorIsTrue() {
    return !frontSensor.get();
  }

  @Override
  public boolean backSensorIsTrue() {
    return !backSensor.get();
  }
}
