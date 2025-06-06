package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRotLeft = 0.0;
    public double velocityRadPerSecLeft = 0.0;
    public double appliedVoltsLeft = 0.0;
    public double currentAmpsLeft = 0.0;

    public double positionRotRight = 0.0;
    public double velocityRadPerSecRight = 0.0;
    public double appliedVoltsRight = 0.0;
    public double currentAmpsRight = 0.0;

    public double positionAlternateEncoder = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocityRadPerSec) {}

  public default void set(double power) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void runPosition(double position) {}

  public default void reset() {}
}
