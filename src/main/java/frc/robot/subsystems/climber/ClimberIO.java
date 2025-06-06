package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPower(double power) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
