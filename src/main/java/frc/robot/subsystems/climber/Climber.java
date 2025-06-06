package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void set(double power) {
    io.setPower(power);
  }

  public void setPowerClimb() {
    io.setPower(ClimberConstants.POWER_TO_CLIMB);
  }

  public void setMinimalPowerClimb() {
    io.setPower(ClimberConstants.MINIMAL_CLIMB);
  }

  public void setReversePowerClimb() {
    io.setPower(ClimberConstants.REVERSE_POWER_TO_CLIMB);
  }

  /** Stops the Climber. */
  public void stop() {
    io.stop();
  }
}
