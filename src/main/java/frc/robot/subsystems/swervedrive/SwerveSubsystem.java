// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.DriveConstants.ZoneLocates;
import frc.robot.subsystems.swervedrive.DriveConstants.ZoneLocates.Zones;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.util.Circle2d;
import frc.robot.util.LocalADStarAK;
// import frc.robot.subsystems.swervedrivedrive.Vision.Cameras;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  /** Swerve drive object. */
  private final SwerveDrive swerveDrive;
  /** Enable vision odometry updates while driving. */
  private final boolean visionDriveTest = true;
  /** PhotonVision class to keep an accurate odometry. */
  // private       Vision      vision;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 51;

  private static final double TIPPING_THRESHOLD = 3;
  private static final double ROBOT_MOI = 4.4679;
  private static final double WHEEL_COF = 1.2;

  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              DriveConstants.kWheelRadius.in(Meters),
              Constants.MAX_SPEED,
              WHEEL_COF,
              DCMotor.getKrakenX60(1).withReduction(DriveConstants.DRIVE_GEAR_RATIO),
              DriveConstants.kSlipCurrent.in(Amps),
              1),
          getModuleTranslations());

  private boolean useMegatag2 = true;

  public SwerveSubsystem(File directory) {
    boolean blueAlliance = false;
    Pose2d startingPose =
        blueAlliance
            ? new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0))
            : new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)), Rotation2d.fromDegrees(180));
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being
    // created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive =
          new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(
        false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(
        false); // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
    // simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(
        true, true,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(
        true, 3); // Enable if you want to resynchronize your absolute encoders and motor encoders
    // periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal
    // encoder and push the offsets onto it. Throws warning if not possible

    swerveDrive.setMaximumAllowableSpeeds(Constants.MAX_SPEED, Units.degreesToRadians(720));

    if (visionDriveTest) {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    setupPathPlanner();
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    swerveDrive.swerveController.addSlewRateLimiters(new SlewRateLimiter(1), null, null);
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(
      SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive =
        new SwerveDrive(
            driveCfg,
            controllerCfg,
            Constants.MAX_SPEED,
            new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.fromDegrees(0)));
  }

  /** Setup the photon vision class. */
  public void setupPhotonVision() {
    // vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  public void setLimitSpeed(boolean state) {
    /*if(state){
      swerveDrive.setMaximumAllowableSpeeds(Constants.MAX_SPEED_LIMITED, Constants.MAX_SPEED_LIMITED);
    }
    else{
      swerveDrive.setMaximumAllowableSpeeds(Constants.MAX_SPEED, Constants.MAX_SPEED);
    }*/
  }

  @Override
  public void periodic() {
    Pose2d visionPose;
    double timestamp;
    LimelightHelpers.SetIMUMode("limelight-left", 0);
    timestamp = LimelightHelpers.getLatency_Capture("limelight-left") 
    + LimelightHelpers.getLatency_Pipeline("limelight-left");
    timestamp = Timer.getFPGATimestamp() - (timestamp / 1000.0);

    if (
      LimelightHelpers.getTV("limelight-left") && 
      swerveDrive.getRobotVelocity().vxMetersPerSecond < 2 &&
      swerveDrive.getRobotVelocity().vyMetersPerSecond < 2 &&
      swerveDrive.getRobotVelocity().omegaRadiansPerSecond < (Math.PI/4) 
    ) {

    if (!useMegatag2) {
      visionPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-left");
    } else {
      if(DriverStation.getAlliance().get()==Alliance.Red){
        LimelightHelpers.SetRobotOrientation("limelight-left", getRotation().getDegrees()-180, 0, 0, 0, 0, 0);
        visionPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-left").pose;
      }
      else{
        LimelightHelpers.SetRobotOrientation("limelight-left", getRotation().getDegrees(), 0, 0, 0, 0, 0);
        visionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left").pose;
      }
    }
        swerveDrive.addVisionMeasurement(visionPose, timestamp);
    }

    // When vision is enabled we must manually update odometry in SwerveDrive
    swerveDrive.updateOdometry();
  }

  @Override
  public void simulationPeriodic() {}

  /** Setup AutoBuilder for PathPlanner. */
  public void setupPathPlanner() {
    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotVelocity,
        this::drive,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(8, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @return A {@link Command} which will run the alignment.
   */
  /*public Command aimAtTarget(Cameras camera)
  {

    return run(() -> {
      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
      if (resultO.isPresent())
      {
        var result = resultO.get();
        if (result.hasTargets())
        {
          drive(getTargetSpeeds(0,
                                0,
                                Rotation2d.fromDegrees(result.getBestTarget()
                                                             .getYaw()))); // Not sure if this will work, more math may be required.
        }
      }
    });
  }*/

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            Constants.MAX_SPEED,
            1.2,
            Units.rotationsPerMinuteToRadiansPerSecond(1/8),
            Units.rotationsPerMinuteToRadiansPerSecond(1/8));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException {
    SwerveSetpointGenerator setpointGenerator =
        new SwerveSetpointGenerator(PP_CONFIG, swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint =
        new AtomicReference<>(
            new SwerveSetpoint(
                swerveDrive.getRobotVelocity(),
                swerveDrive.getStates(),
                DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(
        () -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint =
              setpointGenerator.generateSetpoint(
                  prevSetpoint.get(),
                  robotRelativeChassisSpeed.get(),
                  newTime - previousTime.get());
          swerveDrive.drive(
              newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);
        });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(
      Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(
          () -> {
            return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
          });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
        3.0,
        5.0,
        3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(
            () ->
                swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0))
                    > distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move
          swerveDrive.drive(
              SwerveMath.scaleTranslation(
                  new Translation2d(
                      translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                      translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                  0.8),
              Math.pow(angularRotationX.getAsDouble(), 3)
                  * swerveDrive.getMaximumChassisAngularVelocity(),
              true,
              false);
        });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX Heading X to calculate angle of the joystick.
   * @param headingY Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for
    // this kind of control.
    return run(
        () -> {
          Translation2d scaledInputs =
              SwerveMath.scaleTranslation(
                  new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  scaledInputs.getX(),
                  scaledInputs.getY(),
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()));
        });
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field- and robot-relative
   * modes, which affect how the translation vector is used.
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot, in
   *     meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   *     positive y is torwards port (left). In field-relative mode, positive x is away from the
   *     alliance wall (field North) and positive y is torwards the left wall when looking through
   *     the driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
   *     field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
        translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity with anti-tipping control.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOrientedWithBalance(ChassisSpeeds velocity) {
    double pitch = getPitch().getDegrees();
    double roll = getRoll().getDegrees();

    boolean isTipping = false;

    if (Math.abs(pitch) > TIPPING_THRESHOLD || Math.abs(roll) > TIPPING_THRESHOLD) {
      isTipping = true;
      double tiltAngleRadians = Math.atan2(-roll, -pitch);
      Rotation2d tiltDirection = new Rotation2d(tiltAngleRadians);

      Translation2d correctionVector = new Translation2d(0, 1).rotateBy(tiltDirection);

      Translation2d linearVelocity =
          new Translation2d(
              correctionVector.getX() * Constants.MAX_SPEED,
              correctionVector.getY() * Constants.MAX_SPEED);

      double omega = velocity.omegaRadiansPerSecond;

      velocity = new ChassisSpeeds(-linearVelocity.getX(), linearVelocity.getY(), omega);
    }

    if (isTipping) {
      swerveDrive.drive(velocity);
    } else {
      swerveDrive.driveFieldOriented(velocity);
    }
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  /**
   * Drive the robot given a chassis field oriented velocity with anti-tipping control.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOrientedWithBalance(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          double pitch = getPitch().getDegrees();
          double roll = getRoll().getDegrees();

          ChassisSpeeds currentVelocity = velocity.get();

          boolean isTipping = false;

          if (Math.abs(pitch) > TIPPING_THRESHOLD || Math.abs(roll) > TIPPING_THRESHOLD) {
            isTipping = true;
            double tiltAngleRadians = Math.atan2(-roll, -pitch);
            Rotation2d tiltDirection = new Rotation2d(tiltAngleRadians);

            Translation2d correctionVector = new Translation2d(0, 1).rotateBy(tiltDirection);

            Translation2d linearVelocity =
                new Translation2d(
                    correctionVector.getX() * Constants.MAX_SPEED,
                    correctionVector.getY() * Constants.MAX_SPEED);

            double omega = currentVelocity.omegaRadiansPerSecond;

            currentVelocity =
                new ChassisSpeeds(-linearVelocity.getX(), linearVelocity.getY(), omega);
          }

          if (isTipping) {
            swerveDrive.drive(currentVelocity);
          } else {
            swerveDrive.driveFieldOriented(velocity.get());
          }
        });
  }

  /**
   * Drive according to the chassis robot oriented velocity with anti-tipping control.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void driveWithBalance(ChassisSpeeds velocity) {
    double pitch = getPitch().getDegrees();
    double roll = getRoll().getDegrees();

    if (Math.abs(pitch) > TIPPING_THRESHOLD || Math.abs(roll) > TIPPING_THRESHOLD) {
      double tiltAngleRadians = Math.atan2(-roll, -pitch);
      Rotation2d tiltDirection = new Rotation2d(tiltAngleRadians);

      Translation2d correctionVector = new Translation2d(0, 1).rotateBy(tiltDirection);

      Translation2d linearVelocity =
          new Translation2d(
              correctionVector.getX() * Constants.MAX_SPEED,
              correctionVector.getY() * Constants.MAX_SPEED);

      double omega = velocity.omegaRadiansPerSecond;

      velocity = new ChassisSpeeds(-linearVelocity.getX(), linearVelocity.getY(), omega);
    }
    swerveDrive.drive(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Gets the current rotation of the robot, as reported by IMU.
   *
   * @return The robot's rotation
   */
  @AutoLogOutput(key = "Odometry/Robot Rotation")
  public Rotation2d getRotation() {
    return swerveDrive.getPose().getRotation();
  }

  /** Returns the current odometry field zone. */
  @AutoLogOutput(key = "Odometry/Current Field Zones")
  public Zones[] getCurrentZones() {
    Translation2d robotPosition = getPose().getTranslation();
    List<Zones> detectedZones = new ArrayList<>();

    for (Circle2d circle : ZoneLocates.zones) {
      Translation2d center = circle.getCenter();
      double distance = robotPosition.getDistance(center);

      if (distance <= circle.getRadius()) {
        detectedZones.add(circle.getZone());
      }
    }

    if (detectedZones.isEmpty()) {
      return new Zones[] {Zones.NOT_ZONE};
    }

    return detectedZones.toArray(new Zones[0]);
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   *
   * <p>If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the
   * underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls
   * to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot
   * at an offset of 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.X_DISTANCE_FL, DriveConstants.Y_DISTANCE_FL),
      new Translation2d(DriveConstants.X_DISTANCE_FR, DriveConstants.Y_DISTANCE_FR),
      new Translation2d(DriveConstants.X_DISTANCE_BL, DriveConstants.Y_DISTANCE_BL),
      new Translation2d(DriveConstants.X_DISTANCE_BR, DriveConstants.Y_DISTANCE_BR)
    };
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /** Lock the swerve drive to prevent it from moving. */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  @AutoLogOutput(key = "Odometry/Pitch")
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Gets the current roll angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  @AutoLogOutput(key = "Odometry/Roll")
  public Rotation2d getRoll() {
    return swerveDrive.getRoll();
  }

  /** Add a fake vision reading for testing purposes. */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(
        new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  public void addVisionMeasurement(Pose2d pose, double timestamps) {
    swerveDrive.addVisionMeasurement(pose, timestamps);
  }

  public InstantCommand useMegatag2(boolean use) {
    return new InstantCommand(() -> this.useMegatag2 = use);
  }

  public void setMegatag2(boolean use) {
    useMegatag2 = use;
  }

  public void resetDriveEncoders() {
    swerveDrive.resetDriveEncoders();
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
}
