package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Supplier;

/**
 * AntiTipping corrects robot tipping based on pitch and roll,
 * applying a PID-scaled acceleration in the direction of the inclination.
 */
public class AntiTipping {
  private double tippingThresholdDegrees = 10.0;
  private double maxCorrectionSpeed = 2.0; // m/s

  private final PIDController correctionPID = new PIDController(1.0, 0.0, 0.0);

  private Supplier<Double> pitchSupplier;
  private Supplier<Double> rollSupplier;

  private double lastPitch = 0.0;
  private double lastRoll = 0.0;
  private double lastInclinationMagnitude = 0.0;
  private double lastYawDirectionDeg = 0.0;
  private boolean lastIsTipping = false;
  private Rotation2d lastTiltDirection = new Rotation2d();
  private Translation2d lastCorrectionVector = new Translation2d();

  /**
   * Constructs an AntiTipping instance.
   * @param pitchSupplier function that supplies pitch in degrees
   * @param rollSupplier function that supplies roll in degrees
   */
  public AntiTipping(Supplier<Double> pitchSupplier, Supplier<Double> rollSupplier) {
    this.pitchSupplier = pitchSupplier;
    this.rollSupplier = rollSupplier;
  }

  /** Sets the tipping angle threshold in degrees */
  public void setTippingThreshold(double degrees) {
    this.tippingThresholdDegrees = degrees;
  }

  /** Sets the maximum correction speed in m/s */
  public void setMaxCorrectionSpeed(double speedMetersPerSecond) {
    this.maxCorrectionSpeed = speedMetersPerSecond;
  }

  /** Sets PID constants for correction */
  public void setPIDConstants(double kP, double kI, double kD) {
    correctionPID.setPID(kP, kI, kD);
  }

  /** Sets the tolerance for the PID controller */
  public void setTolerance(double tolerance) {
    correctionPID.setTolerance(tolerance);
  }

  /**
   * Calculates and returns a possibly corrected ChassisSpeeds
   * if tipping is detected.
   * @param desiredSpeeds the desired ChassisSpeeds
   * @return corrected ChassisSpeeds if tipping is detected
   */
  public ChassisSpeeds calculate(ChassisSpeeds desiredSpeeds, Rotation2d robotAngle) {
    double pitch = pitchSupplier.get();
    double roll = rollSupplier.get();
    lastPitch = pitch;
    lastRoll = roll;

    if (Math.abs(pitch) < tippingThresholdDegrees && Math.abs(roll) < tippingThresholdDegrees) {
      lastIsTipping = false;
      lastInclinationMagnitude = 0.0;
      lastYawDirectionDeg = 0.0;
      lastTiltDirection = new Rotation2d();
      lastCorrectionVector = new Translation2d();
      return desiredSpeeds;
    }

    lastIsTipping = true;

    // Calculate the tilt direction
    Rotation2d tiltDirection = new Rotation2d(Math.atan2(-roll, -pitch));
    lastTiltDirection = tiltDirection;
    lastYawDirectionDeg = tiltDirection.getDegrees();

    // Use the magnitude of the inclination as the value for PID
    double tiltMagnitude = Math.hypot(pitch, roll);
    lastInclinationMagnitude = tiltMagnitude;

    // Apply PID directly to tilt magnitude
    double correctionSpeed = correctionPID.calculate(tiltMagnitude, 0.0);
    correctionSpeed = MathUtil.clamp(correctionSpeed, -maxCorrectionSpeed, maxCorrectionSpeed);

    // Create the correction vector
    Translation2d correctionVector = new Translation2d(1, 0)
        .rotateBy(tiltDirection)
        .times(correctionSpeed);

    lastCorrectionVector = correctionVector;

    ChassisSpeeds speeds = new ChassisSpeeds(
      -correctionVector.getX(),
       correctionVector.getY(),
       desiredSpeeds.omegaRadiansPerSecond
    );
    speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotAngle);
    return speeds;
  }

  /** Returns the last pitch value in degrees */
  public double getLastPitch() {
    return lastPitch;
  }

  /** Returns the last roll value in degrees */
  public double getLastRoll() {
    return lastRoll;
  }

  /** Returns the last computed inclination magnitude (hypot of pitch and roll) */
  public double getLastInclinationMagnitude() {
    return lastInclinationMagnitude;
  }

  /** Returns the last tilt direction in degrees (pseudo-yaw) */
  public double getLastYawDirectionDeg() {
    return lastYawDirectionDeg;
  }

  /** Returns whether the system detected tipping during the last update */
  public boolean isTipping() {
    return lastIsTipping;
  }

  /** Returns the last computed tilt direction as Rotation2d */
  public Rotation2d getLastTiltDirection() {
    return lastTiltDirection;
  }

  /** Returns the last correction vector applied as Translation2d */
  public Translation2d getLastCorrectionVector() {
    return lastCorrectionVector;
  }
}
