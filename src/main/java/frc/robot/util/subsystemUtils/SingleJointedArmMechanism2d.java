package frc.robot.util.subsystemUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * A Mechanism2d object to display the current angle and the target angle of a single jointed arm.
 */
public class SingleJointedArmMechanism2d {
  private final String key;
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d currentPositionLigament, targetPositionLigament;

  /**
   * Constructs a SingleJointedArmMechanism2d object.
   *
   * @param key the key of the mechanism
   * @param mechanismColor the color of the mechanism
   */
  public SingleJointedArmMechanism2d(String key, Color mechanismColor) {
    this(key, MechanismConstants.MECHANISM_LINE_LENGTH, mechanismColor);
  }

  /**
   * Constructs a SingleJointedArmMechanism2d object.
   *
   * @param name the name of the mechanism
   * @param armLength the length of the arm
   * @param mechanismColor the color of the mechanism
   */
  public SingleJointedArmMechanism2d(String name, double armLength, Color mechanismColor) {
    this.key = "Mechanisms/" + name;
    final double mechanismMiddle = MechanismConstants.LIGAMENT_END_TO_EDGE_RATIO * armLength;
    this.mechanism = new LoggedMechanism2d(2 * mechanismMiddle, 2 * mechanismMiddle);
    final LoggedMechanismRoot2d root = mechanism.getRoot("Root", mechanismMiddle, mechanismMiddle);
    this.currentPositionLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "ZCurrentPositionLigament",
                armLength,
                0,
                MechanismConstants.MECHANISM_LINE_WIDTH,
                new Color8Bit(mechanismColor)));
    this.targetPositionLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "TargetPositionLigament",
                armLength,
                0,
                MechanismConstants.MECHANISM_LINE_WIDTH,
                MechanismConstants.GRAY));
  }

  /**
   * Updates the mechanism's angle and target angle, then logs the Mechanism2d object.
   *
   * @param currentAngle the current angle
   * @param targetAngle the target angle
   */
  public void update(Rotation2d currentAngle, Rotation2d targetAngle) {
    setTargetAngle(targetAngle);
    update(currentAngle);
  }

  /**
   * Updates the mechanism's angle, then logs the Mechanism2d object.
   *
   * @param currentAngle the current angle
   */
  public void update(Rotation2d currentAngle) {
    setCurrentAngle(currentAngle);
    update();
  }

  /** Logs the Mechanism2d object. */
  public void update() {
    Logger.recordOutput(key, mechanism);
  }

  public void setCurrentAngle(Rotation2d currentAngle) {
    currentPositionLigament.setAngle(currentAngle);
  }

  /**
   * Sets the target angle of the mechanism.
   *
   * @param targetAngle the target angle
   */
  public void setTargetAngle(Rotation2d targetAngle) {
    targetPositionLigament.setAngle(targetAngle);
  }
}
