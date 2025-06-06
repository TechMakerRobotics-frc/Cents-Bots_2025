package frc.robot.subsystems.cradle;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Cradle extends SubsystemBase {

  private final CradleIO io;
  private final CradleIOInputsAutoLogged inputsWrist = new CradleIOInputsAutoLogged();
  private final CradleIOInputsAutoLogged inputsLeft = new CradleIOInputsAutoLogged();
  private final CradleIOInputsAutoLogged inputsRight = new CradleIOInputsAutoLogged();

  public Cradle(CradleIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputsLeft, inputsRight, inputsWrist);
    Logger.processInputs("Cradle/Left", inputsLeft);
    Logger.processInputs("Cradle/Right", inputsRight);
    Logger.processInputs("Cradle/Wrist", inputsWrist);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec);
    Logger.recordOutput("Cradle/SetpointRPM", velocityRPM);
  }

  public void set(double power) {
    io.set(power);
  }

  public void setMoviment(double power) {
    io.setMoviment(power);
  }

  /** Stops the Cradle. */
  public void stop() {
    io.stop();
  }

  public void stopMoviment() {
    io.stopMoviment();
  }

  public void runPosition(double position) {
    if (position > CradleConstants.CRADLE_MAX_POSITION) {
      position = CradleConstants.CRADLE_MAX_POSITION;
    }
    if (position < CradleConstants.CRADLE_MIN_POSITION) {
      position = CradleConstants.CRADLE_MIN_POSITION;
    }

    io.setPosition(position);
  }

  @AutoLogOutput(key = "Cradle/Front Sensor")
  public boolean frontSensorIsTrue() {
    return io.frontSensorIsTrue();
  }

  @AutoLogOutput(key = "Cradle/Back Sensor")
  public boolean backSensorIsTrue() {
    return io.backSensorIsTrue();
  }

  @AutoLogOutput(key = "Cradle/AtSetpoint")
  public boolean atSetpoint() {
    return (Math.abs(inputsWrist.velocityRadPerSec) < 0.1);
  }

  public void setZero() {
    io.setZero();
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public double getEncoder() {
    return io.getEncoder();
  }

  public void setInverted(double power) {
    io.setInvertPower(power);
  }
}
