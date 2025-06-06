package frc.robot.subsystems.funnel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {

  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

  public Funnel(FunnelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);
  }

  public void runVoltsIntakeMotor(double volts) {
    io.setVoltageConveyorMotor(volts);
  }

  public void runVoltsConveyorMotor(double volts) {
    io.setVoltageConveyorMotor(volts);
  }

  public void runVelocityIntakeCoralMotor(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocityIntakeMotor(velocityRadPerSec);
    Logger.recordOutput("IntakeCoral/SetpointRPM", velocityRPM);
  }

  public void runVelocityConveyorMotor(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocityConveyorMotor(velocityRadPerSec);
    Logger.recordOutput("IntakeCoral/SetpointRPM", velocityRPM);
  }

  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public void stopAll() {
    io.stopAll();
  }

  public void stopIntakeCoralMotor() {
    io.stopIntakeMotor();
  }

  public void stopConveyorMotor() {
    io.stopConveyorMotor();
  }

  public void setIntake(double power) {
    io.setIntake(power);
  }

  public void setConveyor(double power) {
    io.setConveyor(power);
  }
}
