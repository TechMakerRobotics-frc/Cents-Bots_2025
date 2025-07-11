package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.Circle2d;
import java.util.List;

public class DriveConstants {
  public static final double kCoupleRatio = 3.5;
  public static final Distance kWheelRadius = Inches.of(2);
  public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
  public static final Current kSlipCurrent = Amps.of(120.0);
  public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;
  public static final double kSteerInertia = 0.01;
  public static final double kDriveInertia = 0.025;
  // Simulated voltage necessary to overcome friction
  public static final Voltage kSteerFrictionVoltage = Volts.of(0.25);
  public static final Voltage kDriveFrictionVoltage = Volts.of(0.25);

  public static final int PIGEON_ID = 14;

  // Modules:
  // Front Left
  public static final int TALON_FL = 1;
  public static final int SPARK_FL = 2;
  public static final int CANCODER_FL = 3;
  public static final double ENCODER_OFFSET_FL = (-0.408203);
  public static final Angle ENCODER_OFFSET_FL_ANGLE = Rotations.of(ENCODER_OFFSET_FL);
  public static final boolean ENCODER_INVERTED_FL = false;
  public static final Distance X_DISTANCE_FL = Inches.of(10.5);
  public static final Distance Y_DISTANCE_FL = Inches.of(10.5);

  // Front Right
  public static final int TALON_FR = 4;
  public static final int SPARK_FR = 5;
  public static final int CANCODER_FR = 6;
  public static final double ENCODER_OFFSET_FR = (0.325195);
  public static final Angle ENCODER_OFFSET_FR_ANGLE = Rotations.of(ENCODER_OFFSET_FR);
  public static final boolean ENCODER_INVERTED_FR = false;
  public static final Distance X_DISTANCE_FR = Inches.of(10.5);
  public static final Distance Y_DISTANCE_FR = Inches.of(-10.5);

  // Back Left
  public static final int TALON_BL = 7;
  public static final int SPARK_BL = 8;
  public static final int CANCODER_BL = 9;
  public static final double ENCODER_OFFSET_BL = (0.389404);
  public static final Angle ENCODER_OFFSET_BL_ANGLE = Rotations.of(ENCODER_OFFSET_BL);
  public static final boolean ENCODER_INVERTED_BL = false;
  public static final Distance X_DISTANCE_BL = Inches.of(-10.5);
  public static final Distance Y_DISTANCE_BL = Inches.of(10.5);

  // Back Right
  public static final int TALON_BR = 10;
  public static final int SPARK_BR = 11;
  public static final int CANCODER_BR = 12;
  public static final double ENCODER_OFFSET_BR = (0.374023);
  public static final Angle ENCODER_OFFSET_BR_ANGLE = Rotations.of(ENCODER_OFFSET_BR);
  public static final boolean ENCODER_INVERTED_BR = false;
  public static final Distance X_DISTANCE_BR = Inches.of(-10.5);
  public static final Distance Y_DISTANCE_BR = Inches.of(-10.5);

  public static final String CANBUS = "CANivore";

  public static final boolean TURN_MOTOR_INVERTED = true;

  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;

  public static final double ODOMETRY_UPDATE_FREQUENCY = 100.0;
  public static final double OTHER_SIGNALS_UPDATE_FREQUENCY = 50.0;

  public static final int CAN_TIMEOUT_MS = 250;
  public static final int SMART_CURRENT_LIMIT = 30;
  public static final double VOLTAGE_COMPENSATION = 10.0;

  public static final double INITIAL_ENCODER_POSITION = 0.0;
  public static final int ENCODER_MEASUREMENT_PERIOD_MS = 10;
  public static final int ENCODER_AVERAGE_DEPTH = 2;

  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(ENCODER_OFFSET_FL);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(ENCODER_OFFSET_FR);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(ENCODER_OFFSET_BL);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(ENCODER_OFFSET_BR);

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
  // Front Left
  public static final int kFrontLeftDriveMotorId = 1;
  public static final int kFrontLeftSteerMotorId = 2;
  public static final int kFrontLeftEncoderId = 3;
  public static final Angle kFrontLeftEncoderOffset = Rotations.of(0.091553);
  public static final boolean kFrontLeftSteerMotorInverted = turnInverted;
  public static final boolean kFrontLeftCANcoderInverted = turnEncoderInverted;

  public static final Distance kFrontLeftXPos = Inches.of(10.5);
  public static final Distance kFrontLeftYPos = Inches.of(10.5);

  // Front Right
  public static final int kFrontRightDriveMotorId = 4;
  public static final int kFrontRightSteerMotorId = 5;
  public static final int kFrontRightEncoderId = 6;
  public static final Angle kFrontRightEncoderOffset = Rotations.of(0.324707);
  public static final boolean kFrontRightSteerMotorInverted = turnInverted;
  public static final boolean kFrontRightCANcoderInverted = turnEncoderInverted;

  public static final Distance kFrontRightXPos = Inches.of(10.5);
  public static final Distance kFrontRightYPos = Inches.of(-10.5);

  // Back Left
  public static final int kBackLeftDriveMotorId = 7;
  public static final int kBackLeftSteerMotorId = 8;
  public static final int kBackLeftEncoderId = 9;
  public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.112549);
  public static final boolean kBackLeftSteerMotorInverted = turnInverted;
  public static final boolean kBackLeftCANcoderInverted = turnEncoderInverted;

  public static final Distance kBackLeftXPos = Inches.of(-10.5);
  public static final Distance kBackLeftYPos = Inches.of(10.5);

  // Back Right
  public static final int kBackRightDriveMotorId = 10;
  public static final int kBackRightSteerMotorId = 11;
  public static final int kBackRightEncoderId = 12;
  public static final Angle kBackRightEncoderOffset = Rotations.of(0.373047);
  public static final boolean kBackRightSteerMotorInverted = turnInverted;
  public static final boolean kBackRightCANcoderInverted = turnEncoderInverted;

  public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  public static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a
                  // relatively
                  // low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));
  public static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

  public static final boolean kInvertLeftSide = false;
  public static final boolean kInvertRightSide = true;

  public static final Distance kBackRightXPos = Inches.of(-10.5);
  public static final Distance kBackRightYPos = Inches.of(-10.5);

  public static class ZoneLocates {
    public static final Circle2d blueLeftStation =
        new Circle2d(1.100, 7.124, 1.4, Zones.BLUE_LEFT_STATION);
    public static final Circle2d blueRightStation =
        new Circle2d(1.100, 0.974, 1.4, Zones.BLUE_RIGHT_STATION);
    public static final Circle2d redLeftStation =
        new Circle2d(16.697198, 0.974, 1.4, Zones.RED_LEFT_STATION);
    public static final Circle2d redRightStation =
        new Circle2d(16.411, 6.968, 1.4, Zones.RED_RIGHT_STATION);
    public static final Circle2d blueProcessor =
        new Circle2d(6.341, 0.471, 1, Zones.BLUE_PROCESSOR);
    public static final Circle2d redProcessor = new Circle2d(11.540, 7.651, 1, Zones.RED_PROCESSOR);
    public static final Circle2d blueReefSecurityZone =
        new Circle2d(4.486, 4.031, 1.54, Zones.REEF_SECURITY_ZONE_BLUE);
    public static final Circle2d redReefSecurityZone =
        new Circle2d(13.061, 4.031, 1.54, Zones.REEF_SECURITY_ZONE_RED);
    public static final Circle2d blueReefSubsystemZone =
        new Circle2d(4.486, 4.031, 2.54, Zones.REEF_SUBSYSTEM_ZONE_BLUE);
    public static final Circle2d redReefSubsystemZone =
        new Circle2d(13.061, 4.031, 2.54, Zones.REEF_SUBSYSTEM_ZONE_RED);

    // Lista contendo todas as zonas
    public static final List<Circle2d> zones =
        List.of(
            blueLeftStation,
            blueRightStation,
            redLeftStation,
            redRightStation,
            blueProcessor,
            redProcessor,
            blueReefSecurityZone,
            redReefSecurityZone,
            blueReefSubsystemZone,
            redReefSubsystemZone);

    public static enum Zones {
      NOT_ZONE,
      BLUE_LEFT_STATION,
      BLUE_RIGHT_STATION,
      RED_LEFT_STATION,
      RED_RIGHT_STATION,
      BLUE_PROCESSOR,
      RED_PROCESSOR,
      REEF_SECURITY_ZONE_BLUE,
      REEF_SECURITY_ZONE_RED,
      REEF_SUBSYSTEM_ZONE_BLUE,
      REEF_SUBSYSTEM_ZONE_RED
    }
  }

  // Gear ratios for SDS MK4i L3, adjust as necessary
  public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  public static final Slot0Configs STEER_GAINS =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(2.0)
          .withKS(0.2)
          .withKV(1.59)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  public static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs().withKP(0.03).withKI(0).withKD(0).withKS(0).withKV(0.124);

  public static final SwerveModuleConstantsFactory<
          ParentConfiguration, ParentConfiguration, ParentConfiguration>
      ConstantCreatorSIM =
          new SwerveModuleConstantsFactory<>()
              .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
              .withSteerMotorGearRatio(TURN_GEAR_RATIO)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorGains(STEER_GAINS)
              .withDriveMotorGains(DRIVE_GAINS)
              .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
              .withSlipCurrent(kSlipCurrent)
              .withSpeedAt12Volts(Constants.MAX_SPEED)
              .withFeedbackSource(kSteerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(cancoderInitialConfigs)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage);

  public static final SwerveModuleConstants<
          ParentConfiguration, ParentConfiguration, ParentConfiguration>
      FrontLeftSIM =
          ConstantCreatorSIM.createModuleConstants(
              SPARK_FL,
              TALON_FL,
              CANCODER_FL,
              ENCODER_OFFSET_FL_ANGLE,
              X_DISTANCE_FL,
              Y_DISTANCE_FL,
              kInvertLeftSide,
              TURN_MOTOR_INVERTED,
              ENCODER_INVERTED_FL);
  public static final SwerveModuleConstants<
          ParentConfiguration, ParentConfiguration, ParentConfiguration>
      FrontRightSIM =
          ConstantCreatorSIM.createModuleConstants(
              TALON_FR,
              SPARK_FR,
              CANCODER_FR,
              ENCODER_OFFSET_FR_ANGLE,
              X_DISTANCE_FR,
              Y_DISTANCE_FR,
              kInvertRightSide,
              TURN_MOTOR_INVERTED,
              ENCODER_INVERTED_FL);
  public static final SwerveModuleConstants<
          ParentConfiguration, ParentConfiguration, ParentConfiguration>
      BackLeftSIM =
          ConstantCreatorSIM.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              kBackLeftXPos,
              kBackLeftYPos,
              kInvertLeftSide,
              kBackLeftSteerMotorInverted,
              kBackLeftCANcoderInverted);
  public static final SwerveModuleConstants<
          ParentConfiguration, ParentConfiguration, ParentConfiguration>
      BackRightSIM =
          ConstantCreatorSIM.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              kBackRightXPos,
              kBackRightYPos,
              kInvertRightSide,
              kBackRightSteerMotorInverted,
              kBackRightCANcoderInverted);
}
