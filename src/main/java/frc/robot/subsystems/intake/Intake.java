package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputsIntake = new IntakeIOInputsAutoLogged();
  private final IntakeIOInputsAutoLogged inputsRoller = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputsIntake, inputsRoller);
    Logger.processInputs("Intake/Intake motor", inputsIntake);
    Logger.processInputs("Intake/Roller motor", inputsRoller);
  }

  public void runVoltsIntakeMotor(double volts) {
    io.setVoltageRollerMotor(volts);
  }

  public void runVoltsRollerMotor(double volts) {
    io.setVoltageRollerMotor(volts);
  }

  public void runVelocityIntakeMotor(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocityIntakeMotor(velocityRadPerSec);
    Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
  }

  public void runVelocityRollerMotor(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocityRollerMotor(velocityRadPerSec);
    Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
  }

  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputsRoller.velocityRadPerSec);
  }

  public void stopAll() {
    io.stopAll();
  }

  public void stopIntakeMotor() {
    io.stopIntakeMotor();
  }

  public void stopRollerMotor() {
    io.stopIntakeMotor();
  }

  public void setIntake(double power) {
    io.setIntake(power);
  }

  public void setRoller(double power) {
    io.setRoller(power);
  }

  public void runPosition(double position) {
    io.runPosition(position);
  }
}
