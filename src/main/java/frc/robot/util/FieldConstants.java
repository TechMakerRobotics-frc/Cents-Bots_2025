package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FieldConstants {

  /**
   * Provides predefined poses for the reef on the field.
   *
   * <p>The reef is a set of positions where game elements may be located. Each pose is defined for
   * both the blue and red alliances, with red alliance poses obtained by mirroring the blue
   * alliance poses.
   */
  public static class ReefPoses {

    /** Position A on the blue alliance side. */
    public static final Pose2d A_BLUE =
        new Pose2d(3.035, 4.14, new Rotation2d(Units.degreesToRadians(0)));

    /** Position B on the blue alliance side. */
    public static final Pose2d B_BLUE =
        new Pose2d(
            3.035, 3.83, new Rotation2d(Units.degreesToRadians(0)));

    /** Position C on the blue alliance side. */
    public static final Pose2d C_BLUE =
        new Pose2d(3.739783525466919, 3.0643150806427, new Rotation2d(Units.degreesToRadians(60)));

    /** Position D on the blue alliance side. */
    public static final Pose2d D_BLUE =
        new Pose2d(
            4.024022579193115, 2.8956146240234375, new Rotation2d(Units.degreesToRadians(60)));

    /** Position E on the blue alliance side. */
    public static final Pose2d E_BLUE =
        new Pose2d(
            4.949123382568359, 2.9001970291137695, new Rotation2d(Units.degreesToRadians(120)));

    /** Position F on the blue alliance side. */
    public static final Pose2d F_BLUE =
        new Pose2d(
            5.237850189208984, 3.062143087387085, new Rotation2d(Units.degreesToRadians(120)));

    /** Position G on the blue alliance side. */
    public static final Pose2d G_BLUE =
        new Pose2d(
            5.699155807495117, 3.8621926307678223, new Rotation2d(Units.degreesToRadians(180)));

    /** Position H on the blue alliance side. */
    public static final Pose2d H_BLUE =
        new Pose2d(
            5.699155807495117, 4.193874835968018, new Rotation2d(Units.degreesToRadians(180)));

    /** Position I on the blue alliance side. */
    public static final Pose2d I_BLUE =
        new Pose2d(
            5.235140800476074, 4.987342834472656, new Rotation2d(Units.degreesToRadians(-120)));

    /** Position J on the blue alliance side. */
    public static final Pose2d J_BLUE =
        new Pose2d(
            4.950402736663818, 5.152055263519287, new Rotation2d(Units.degreesToRadians(-120)));

    /** Position K on the blue alliance side. */
    public static final Pose2d K_BLUE =
        new Pose2d(
            4.027472972869873, 5.154789447784424, new Rotation2d(Units.degreesToRadians(-60)));

    /** Position L on the blue alliance side. */
    public static final Pose2d L_BLUE =
        new Pose2d(
            3.7414097785949707, 4.986670970916748, new Rotation2d(Units.degreesToRadians(-60)));

    /** Position AB on the blue alliance side. */
    public static final Pose2d AB_BLUE =
        new Pose2d(2.579, 4.025, new Rotation2d(Units.degreesToRadians(0)));

    /** Position CD on the blue alliance side. */
    public static final Pose2d CD_BLUE =
        new Pose2d(3.532, 2.374, new Rotation2d(Units.degreesToRadians(60)));

    /** Position EF on the blue alliance side. */
    public static final Pose2d EF_BLUE =
        new Pose2d(5.443, 2.375, new Rotation2d(Units.degreesToRadians(120)));

    /** Position GH on the blue alliance side. */
    public static final Pose2d GH_BLUE =
        new Pose2d(6.399, 4.028, new Rotation2d(Units.degreesToRadians(180)));

    /** Position IJ on the blue alliance side. */
    public static final Pose2d IJ_BLUE =
        new Pose2d(5.443, 5.676, new Rotation2d(Units.degreesToRadians(-120)));

    /** Position KL on the blue alliance side. */
    public static final Pose2d KL_BLUE =
        new Pose2d(3.534, 5.677, new Rotation2d(Units.degreesToRadians(-60)));

    public static final List<Pose2d> RIGHT_SIDE_BRANCHES_BLUE = List.of(
        ReefPoses.A_BLUE,
        ReefPoses.C_BLUE,
        ReefPoses.E_BLUE,
        ReefPoses.G_BLUE,
        ReefPoses.I_BLUE,
        ReefPoses.K_BLUE
    );
        
    public static final List<Pose2d> LEFT_SIDE_BRANCHES_BLUE = List.of(
        ReefPoses.B_BLUE,
        ReefPoses.D_BLUE,
        ReefPoses.F_BLUE,
        ReefPoses.H_BLUE,
        ReefPoses.J_BLUE,
        ReefPoses.L_BLUE
    );    

    /** Position A on the red alliance side (mirrored from A_BLUE). */
    public static final Pose2d A_RED = GeomUtil.flip(A_BLUE);

    /** Position B on the red alliance side (mirrored from B_BLUE). */
    public static final Pose2d B_RED = GeomUtil.flip(B_BLUE);

    /** Position C on the red alliance side (mirrored from C_BLUE). */
    public static final Pose2d C_RED = GeomUtil.flip(C_BLUE);

    /** Position D on the red alliance side (mirrored from D_BLUE). */
    public static final Pose2d D_RED = GeomUtil.flip(D_BLUE);

    /** Position E on the red alliance side (mirrored from E_BLUE). */
    public static final Pose2d E_RED = GeomUtil.flip(E_BLUE);

    /** Position F on the red alliance side (mirrored from F_BLUE). */
    public static final Pose2d F_RED = GeomUtil.flip(F_BLUE);

    /** Position G on the red alliance side (mirrored from G_BLUE). */
    public static final Pose2d G_RED = GeomUtil.flip(G_BLUE);

    /** Position H on the red alliance side (mirrored from H_BLUE). */
    public static final Pose2d H_RED = GeomUtil.flip(H_BLUE);

    /** Position I on the red alliance side (mirrored from I_BLUE). */
    public static final Pose2d I_RED = GeomUtil.flip(I_BLUE);

    /** Position J on the red alliance side (mirrored from J_BLUE). */
    public static final Pose2d J_RED = GeomUtil.flip(J_BLUE);

    /** Position K on the red alliance side (mirrored from K_BLUE). */
    public static final Pose2d K_RED = GeomUtil.flip(K_BLUE);

    /** Position L on the red alliance side (mirrored from L_BLUE). */
    public static final Pose2d L_RED = GeomUtil.flip(L_BLUE);

    /** Position AB on the red alliance side. */
    public static final Pose2d AB_RED = GeomUtil.flip(AB_BLUE);

    /** Position CD on the red alliance side. */
    public static final Pose2d CD_RED = GeomUtil.flip(CD_BLUE);

    /** Position EF on the red alliance side. */
    public static final Pose2d EF_RED = GeomUtil.flip(EF_BLUE);

    /** Position GH on the red alliance side. */
    public static final Pose2d GH_RED = GeomUtil.flip(GH_BLUE);

    /** Position IJ on the red alliance side. */
    public static final Pose2d IJ_RED = GeomUtil.flip(IJ_BLUE);

    /** Position KL on the red alliance side. */
    public static final Pose2d KL_RED = GeomUtil.flip(KL_BLUE);
  }

  /**
   * Provides predefined poses for the coral station on the field.
   *
   * <p>The coral station is a designated area where specific field interactions occur. Poses are
   * defined for both the blue and red alliances.
   */
  public static class CoralStationPoses {

    /** The left coral station pose for the blue alliance. */
    public static final Pose2d LEFT_BLUE =
        new Pose2d(
            1.6078553199768066, 7.518975257873535, new Rotation2d(Units.degreesToRadians(-54)));

    /** The right coral station pose for the blue alliance. */
    public static final Pose2d RIGHT_BLUE =
        new Pose2d(
            1.464130163192749, 0.6082469820976257, new Rotation2d(Units.degreesToRadians(54)));

    /** The left coral station pose for the red alliance (mirrored from LEFT_BLUE). */
    public static final Pose2d LEFT_RED = GeomUtil.flip(LEFT_BLUE);

    /** The right coral station pose for the red alliance (mirrored from RIGHT_BLUE). */
    public static final Pose2d RIGHT_RED = GeomUtil.flip(RIGHT_BLUE);
  }

  /**
   * Provides predefined poses for the processor on the field.
   *
   * <p>The {@code BLUE} pose refers to the location on the field on the blue alliance side, not
   * where the blue alliance scores. The {@code RED} pose is the mirrored version of {@code BLUE}
   * for the red alliance side.
   */
  public static class ProcessorPoses {

    /**
     * The processor pose on the blue alliance side of the field (not the blue alliance scoring
     * location).
     */
    public static final Pose2d BLUE =
        new Pose2d(
            6.343825817108154, 0.4128298759460449, new Rotation2d(Units.degreesToRadians(-90)));

    /** The processor pose on the red alliance side, obtained by mirroring the blue pose. */
    public static final Pose2d RED = GeomUtil.flip(BLUE);
  }

  public static class StartPoses {
    public static final Pose2d CENTER_BLUE =
        new Pose2d(7.270, Units.inchesToMeters(158.5), Rotation2d.fromDegrees(180));

    public static final Pose2d LEFT_BLUE = new Pose2d();

    public static final Pose2d RIGHT_BLUE = new Pose2d();

    public static final Pose2d CENTER_RED = GeomUtil.flip(CENTER_BLUE);

    public static final Pose2d LEFT_RED = new Pose2d();

    public static final Pose2d RIGHT_RED = new Pose2d();
  }

  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefHeight, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    static {
      // Initialize faces
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefHeight.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L1(Units.inchesToMeters(18), 0),
    L2(Units.inchesToMeters(31.875), -35),
    L3(Units.inchesToMeters(47.625), -35),
    L4(Units.inchesToMeters(72), -90);

    ReefHeight(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public static ReefHeight fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L4);
    }

    public final double height;
    public final double pitch;
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final int aprilTagCount = 22;

  public record CoralObjective(int branchId, ReefHeight reefLevel) {}
}
