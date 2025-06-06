package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;
import frc.robot.interfaces.motor.MotorIOSparkMax;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIOSpark implements ArmIO {

  private final MotorIO motor;
  private LoggedTunableNumber kp = new LoggedTunableNumber("/Arm/P", ArmConstants.ARM_kP);
  private LoggedTunableNumber ki = new LoggedTunableNumber("/Arm/I", ArmConstants.ARM_kI);
  private LoggedTunableNumber kd = new LoggedTunableNumber("/Arm/D", ArmConstants.ARM_kD);
  private LoggedTunableNumber kF = new LoggedTunableNumber("/Arm/F", ArmConstants.ARM_kF);

  public ArmIOSpark() {
    motor =
        new MotorIOSparkMax(
            ArmConstants.motorID,
            MotorType.kBrushless,
            true,
            250,
            10.0,
            80,
            IdleMode.kBrake,
            0.014705000445246696,
            1,
            false,
            0,
            1,
            false);

    motor.configurePIDF(
        ArmConstants.ARM_kP, ArmConstants.ARM_kI, ArmConstants.ARM_kD, ArmConstants.ARM_kF);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    MotorIOInputs motorIOInputs = motor.getMotorIOInputs();
    inputs.appliedVolts = motorIOInputs.appliedVolts;
    inputs.currentAmps = motorIOInputs.currentAmps[0];
    inputs.velocityRadPerSec = motorIOInputs.velocityRadPerSec;
    inputs.positionRad = Units.rotationsToRadians(motorIOInputs.positionRot);
    inputs.positionRot = motorIOInputs.positionRot;
    if (kp.hasChanged(hashCode())
        || ki.hasChanged(hashCode())
        || kd.hasChanged(hashCode())
        || kF.hasChanged(hashCode())) {
      motor.configurePIDF(kp.get(), ki.get(), kd.get(), kF.get());
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    motor.setVelocity(velocityRadPerSec);
  }

  @Override
  public void stop() {
    motor.stop();
  }

  @AutoLogOutput
  @Override
  public double getAbsoluteEncoderRot() {
    return motor.getAbsEncoderPosition();
  }

  @Override
  public void configurePIDF(double kP, double kI, double kD, double kF) {
    motor.configurePIDF(kP, kI, kD, kF);
  }

  @Override
  public void setOffset(double offset) {
    motor.setOffset(offset);
  }

  @Override
  public void set(double power) {
    motor.set(power);
  }

  @Override
  public void runPosition(double position) {
    motor.setPosition(position);
  }
}
