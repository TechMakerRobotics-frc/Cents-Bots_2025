package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;
import frc.robot.interfaces.motor.MotorIOSparkMax;

public class ClimberIOSpark implements ClimberIO {

  private final MotorIO motor;

  public ClimberIOSpark() {
    motor =
        new MotorIOSparkMax(
            ClimberConstants.MOTOR_ID,
            MotorType.kBrushless,
            false,
            250,
            10.0,
            40,
            IdleMode.kBrake,
            1,
            1,
            false,
            -1,
            1,
            false);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    MotorIOInputs motorIOInputs = motor.getMotorIOInputs();
    inputs.appliedVolts = motorIOInputs.appliedVolts;
    inputs.currentAmps = motorIOInputs.currentAmps[0];
    inputs.velocityRadPerSec = motorIOInputs.velocityRadPerSec;
    inputs.positionRad = Units.rotationsToRadians(motorIOInputs.positionRot);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setPower(double power) {
    motor.set(power);
  }

  @Override
  public void stop() {
    motor.stop();
  }
}
