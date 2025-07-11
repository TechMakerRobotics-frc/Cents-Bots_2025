package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double positionRad = 0.0;
    public double positionRot = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocityRadPerSec) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void configurePIDF(double kP, double kI, double kD, double kF) {}

  public default double getAbsoluteEncoderRot() {
    return 0.0;
  }

  public default void setOffset(double offset) {}

  public default void set(double power) {}

  public default void runPosition(double position) {}
}
