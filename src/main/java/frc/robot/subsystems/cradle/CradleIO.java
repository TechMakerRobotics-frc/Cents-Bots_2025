package frc.robot.subsystems.cradle;

import org.littletonrobotics.junction.AutoLog;

public interface CradleIO {
  @AutoLog
  public static class CradleIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double positionRot = 0.0;
    public double AbsolutePosition = 0.0;
  }

  public default void updateInputs(
      CradleIOInputs motorLeftinputs,
      CradleIOInputs motorRightinputs,
      CradleIOInputs motorWristinputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocityRadPerSec) {}

  public default void stop() {}

  public default void stopMoviment() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void set(double power) {}

  public default void setMoviment(double power) {}

  public default void setPosition(double position) {}

  public default boolean frontSensorIsTrue() {
    return false;
  }

  public default boolean backSensorIsTrue() {
    return false;
  }

  public default void setZero() {}

  public default void resetEncoder() {}

  public default double getEncoder() {
    return 0;
  }

  public default void setInvertPower(double power) {}
}
