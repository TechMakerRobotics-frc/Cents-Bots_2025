package frc.robot.commands;

public final class CommandConstants {

  public static final class AlignConstants {
    public static final double VX_P = 0.015;
    public static final double VX_I = 0.0;
    public static final double VX_D = 0.0;

    public static final double VY_P = 0.01;
    public static final double VY_I = 0.0;
    public static final double VY_D = 0.0;

    public static final double V_OMEGA_P = 0.008;
    public static final double V_OMEGA_I = 0.0;
    public static final double V_OMEGA_D = 0.0;

    public static final double VX_BALL_P = 0.02;
    public static final double VX_BALL_I = 0.0;
    public static final double VX_BALL_D = 0.001;

    public static final double VY_BALL_P = 0.015;
    public static final double VY_BALL_I = 0.0;
    public static final double VY_BALL_D = 0.001;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34; // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16; // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;
  }

  public static final class MoveXConstants {
    public static final double k_P = 2;
    public static final double k_I = 0.01;
    public static final double k_D = 0;
  }

  public static final class MoveYConstants {
    public static final double k_P = 2;
    public static final double k_I = 0.01;
    public static final double k_D = 0;
  }

  public static final class MoveHConstants {
    public static final double k_P = 0.1;
    public static final double k_I = 0;
    public static final double k_D = 0;
  }

  public static final class FlywheelConstants {
    public static final double TIME_TO_SHOOT = 1.5; // in seconds
  }
}
