package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec);
    Logger.recordOutput("Arm/SetpointRPM", velocityRPM);
  }

  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public void set(double power) {
    io.set(power);
  }

  /** Stops the Arm. */
  public void stop() {
    io.stop();
  }

  public void runPosition(double position) {
    if (position > ArmConstants.ARM_MAX_POSITION) {
      position = ArmConstants.ARM_MAX_POSITION;
    }
    if (position < ArmConstants.ARM_MIN_POSITION) {
      position = ArmConstants.ARM_MIN_POSITION;
    }
    io.runPosition(position);
  }

  @AutoLogOutput(key = "Arm/AtSetpoint")
  public boolean atSetpoint() {
    return (Math.abs(inputs.velocityRadPerSec) < 0.1);
  }

  public boolean atZero() {
    return (Math.abs(inputs.positionRot) < 0.01);
  }

  public double getPosition() {
    return inputs.positionRot;
  }

  public boolean atLimit() {
    return inputs.currentAmps > 40;
  }

  public void zero() {
    io.setOffset(0);
  }
}
