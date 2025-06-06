package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0, // Camera 3
        1,
        0 // Camera 4
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static final String BL_CAM_NAME = "BackLeft";
  public static final double BL_CAM_X = -0.247655;
  public static final double BL_CAM_Y = -0.227649;
  public static final double BL_CAM_Z = 0.195903;
  public static final double BL_CAM_ROLL = 0.0;
  public static final double BL_CAM_PITCH = -30.0;
  public static final double BL_CAM_YAW = 120.0;

  public static final String BR_CAM_NAME = "BackRight";
  public static final double BR_CAM_X = -0.242623;
  public static final double BR_CAM_Y = 0.223648;
  public static final double BR_CAM_Z = 0.192416;
  public static final double BR_CAM_ROLL = 0.0;
  public static final double BR_CAM_PITCH = -30.0;
  public static final double BR_CAM_YAW = -120;

  public static final String L_CAM_NAME = "LCam";
  public static final double L_CAM_X = 0.034801;
  public static final double L_CAM_Y = 0.315937;
  public static final double L_CAM_Z = 0.980405;
  public static final double L_CAM_ROLL = -16;
  public static final double L_CAM_PITCH = 45.0;
  public static final double L_CAM_YAW = -14.2645;

  public static final String R_CAM_NAME = "RCam";
  public static final double R_CAM_X = 0.034801;
  public static final double R_CAM_Y = -0.315937;
  public static final double R_CAM_Z = 0.980405;
  public static final double R_CAM_ROLL = 12;
  public static final double R_CAM_PITCH = 45.0;
  public static final double R_CAM_YAW = 14.2645;

  public static final String LIMELIGHT_NAME = "Limelight";
  public static final double LIMELIGHT_X = 0.155;
  public static final double LIMELIGHT_Y = 0;
  public static final double LIMELIGHT_Z = 0.545;
  public static final double LIMELIGHT_ROLL = 0;
  public static final double LIMELIGHT_PITCH = -25;
  public static final double LIMELIGHT_YAW = 180;

  public static final Transform3d ROBOT_TO_BL_CAM =
      new Transform3d(
          new Translation3d(BL_CAM_X, BL_CAM_Y, BL_CAM_Z),
          new Rotation3d(
              Units.degreesToRadians(BL_CAM_ROLL),
              Units.degreesToRadians(BL_CAM_PITCH),
              Units.degreesToRadians(BL_CAM_YAW)));

  public static final Transform3d ROBOT_TO_BR_CAM =
      new Transform3d(
          new Translation3d(BR_CAM_X, BR_CAM_Y, BR_CAM_Z),
          new Rotation3d(
              Units.degreesToRadians(BR_CAM_ROLL),
              Units.degreesToRadians(BR_CAM_PITCH),
              Units.degreesToRadians(BR_CAM_YAW)));

  public static final Transform3d ROBOT_TO_L_CAM =
      new Transform3d(
          new Translation3d(L_CAM_X, L_CAM_Y, L_CAM_Z),
          new Rotation3d(
              Units.degreesToRadians(L_CAM_ROLL),
              Units.degreesToRadians(L_CAM_PITCH),
              Units.degreesToRadians(L_CAM_YAW)));

  public static final Transform3d ROBOT_TO_R_CAM =
      new Transform3d(
          new Translation3d(R_CAM_X, R_CAM_Y, R_CAM_Z),
          new Rotation3d(
              Units.degreesToRadians(R_CAM_ROLL),
              Units.degreesToRadians(R_CAM_PITCH),
              Units.degreesToRadians(R_CAM_YAW)));

  public static final Transform3d ROBOT_TO_LIMELIGHT =
      new Transform3d(
          new Translation3d(LIMELIGHT_X, LIMELIGHT_Y, LIMELIGHT_Z),
          new Rotation3d(
              Units.degreesToRadians(LIMELIGHT_ROLL),
              Units.degreesToRadians(LIMELIGHT_PITCH),
              Units.degreesToRadians(LIMELIGHT_YAW)));
}
