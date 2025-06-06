package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void set(double power) {
    io.set(power);
    Logger.recordOutput("Elevator/SetpointRPM", power);
  }

  /** Stops the Elevator. */
  public void stop() {
    io.stop();
  }

  public void reset() {
    io.reset();
  }

  public void runPosition(double position) {
    if (position > ElevatorConstants.ELEVATOR_MAX_POSITION) {
      position = ElevatorConstants.ELEVATOR_MAX_POSITION;
    }
    if (position < ElevatorConstants.ELEVATOR_MIN_POSITION) {
      position = ElevatorConstants.ELEVATOR_MIN_POSITION;
    }
    io.runPosition(position);
  }

  @AutoLogOutput(key = "Elevator/AtSetpoint")
  public boolean atSetpoint() {
    return (Math.abs(inputs.velocityRadPerSecLeft) < 0.1)
        && (Math.abs(inputs.velocityRadPerSecRight) < 0.1);
  }

  public boolean atZero() {
    return (Math.abs(inputs.positionRotLeft) < 15) && (Math.abs(inputs.positionRotRight) < 15);
  }

  public boolean atLimit() {
    return inputs.currentAmpsLeft > 40 || inputs.currentAmpsRight > 40;
  }

  public void zero() {
    io.reset();
  }
}
