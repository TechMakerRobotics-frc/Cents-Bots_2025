package frc.robot.subsystems.cradle;

public class CradleConstants {
  public static final int MOTOR_WRIST_ID = 22;
  public static final int MOTOR_LEFT_ID = 23;
  public static final int MOTOR_RIGHT_ID = 24;

  public static final int FRONT_SENSOR_PORT = 4;
  public static final int BACK_SENSOR_PORT = 5;

  public static final double OFFSET = 0.48;
  public static final double OUT_POSITION = 1.;
  public static final double INTAKE_POWER = 1;
  public static final double INTAKE_BACK = -0.2;
  public static final double INTAKE_ADJUST = 0.1;
  public static final double INTAKE_OUT = 0.3;

  public static double CRADLE_MAX_POSITION = 0.5;
  public static double CRADLE_MIN_POSITION = -0.5;
  public static double CRADLE_POSITION_L1 = 0.0;
  public static double CRADLE_POSITION_L2 = 0.2;
  public static double CRADLE_POSITION_L3 = 0.35;
  public static double CRADLE_POSITION_L4 = 0.35;

  public static double WRIST_kP = 2;
  public static double WRIST_kI = 0.0015;
  public static double WRIST_kD = 0.025;
}
