package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputsIntake, IntakeIOInputs inputsRoller) {}

  public default void setVoltageIntakeMotor(double volts) {}

  public default void setVoltageRollerMotor(double volts) {}

  public default void setVelocityIntakeMotor(double velocityRadPerSec) {}

  public default void setVelocityRollerMotor(double velocityRadPerSec) {}

  public default void stopAll() {}

  public default void stopIntakeMotor() {}

  public default void stopRollerMotor() {}

  public default void stopRoller() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void resetPosition() {}

  public default void setRoller(double power) {}

  public default void setIntake(double power) {}

  public default void runPosition(double position) {}
}
