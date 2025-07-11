package frc.robot.interfaces.motor;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class MotorIOSparkMax implements MotorIO {

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absEncoder;
  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  private SparkClosedLoopController closedLoopController;
  private boolean invertedEncoder;

  public MotorIOSparkMax(
      int id,
      SparkBase.MotorType type,
      boolean inverted,
      int timeout,
      double voltageCompensation,
      int smartCurrentLimit,
      SparkBaseConfig.IdleMode idleMode,
      double positionConversionFactor,
      double velocityConversionFactor,
      boolean invertedEncoder,
      double positionMinInput,
      double positionMaxInput,
      boolean positionWrapEnable) {
    motor = new SparkMax(id, type);

    motor.setCANTimeout(timeout);

    encoder = motor.getEncoder();
    absEncoder = motor.getAbsoluteEncoder();

    motorConfig
        .inverted(inverted)
        .voltageCompensation(voltageCompensation)
        .smartCurrentLimit(smartCurrentLimit)
        .idleMode(idleMode);
    motorConfig
        .encoder
        .positionConversionFactor(positionConversionFactor)
        .velocityConversionFactor(velocityConversionFactor);
    // Enable limit switches to stop the motor when they are closed
    motorConfig
        .limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false);

    // Set the soft limits to stop the motor at -50 and 50 rotations
    motorConfig
        .softLimit
        .forwardSoftLimit(50)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimit(-50)
        .reverseSoftLimitEnabled(false);
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(6)
        .i(0)
        .d(0.1)
        .outputRange(-1, 1)
        .positionWrappingEnabled(positionWrapEnable)
        .positionWrappingInputRange(positionMinInput, positionMaxInput)
        // Set PID values for velocity control in slot 1
        .p(0.01, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = motor.getClosedLoopController();
    this.invertedEncoder = invertedEncoder;
  }

  /**
   * @param inputs
   */
  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.positionRot = encoder.getPosition() * (invertedEncoder ? -1 : 1);
    inputs.positionAlternateEncoder = motor.getEncoder().getPosition() * (invertedEncoder ? -1 : 1);
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
  }

  @Override
  public void set(double power) {
    motor.set(power);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    double speed = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec);
    closedLoopController.setReference(
        speed, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
  }

  @Override
  public void setPosition(double position) {
    closedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    motorConfig.closedLoop.pid(kP, kI, kD);
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void configurePIDF(double kP, double kI, double kD, double kF) {
    motorConfig.closedLoop.pidf(kP, kI, kD, kF);
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void resetPosition() {
    encoder.setPosition(0);
  }

  @Override
  public void setOffset(double offset) {
    encoder.setPosition(offset);
  }

  @Override
  public void resetOffset() {
    encoder.setPosition(0);
    setPosition(0);
  }

  @Override
  public void setBrakeMode(boolean set) {
    var config = new SparkMaxConfig();
    config.idleMode(set ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public double getAbsEncoderPosition() {
    return absEncoder.getPosition();
  }

  @Override
  public double getAbsEncoderVelocityRotPMin() {
    return absEncoder.getVelocity();
  }

  @Override
  public void setFollower(int leader) {
    motorConfig.follow(leader);
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setAbsoluteEncoderPosition() {
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(6)
        .i(0)
        .d(0.1)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-0.5, 0.5)
        // Set PID values for velocity control in slot 1
        .p(0.01, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = motor.getClosedLoopController();
  }
}
