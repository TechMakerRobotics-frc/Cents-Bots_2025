package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void setVoltageIntakeMotor(double volts) {}

  public default void setVoltageConveyorMotor(double volts) {}

  public default void setVelocityIntakeMotor(double velocityRadPerSec) {}

  public default void setVelocityConveyorMotor(double velocityRadPerSec) {}

  public default void stopAll() {}

  public default void stopIntakeMotor() {}

  public default void stopConveyorMotor() {}

  public default void stopConveyor() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void resetPosition() {}

  public default void setIntake(double power) {}

  public default void setConveyor(double power) {}
}
