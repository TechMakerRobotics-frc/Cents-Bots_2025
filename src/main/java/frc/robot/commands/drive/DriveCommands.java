package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 8.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static SlewRateLimiter xLimiter = new SlewRateLimiter(2);

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(
                  xLimiter.calculate(xSupplier.getAsDouble()), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * Constants.MAX_SPEED,
                  linearVelocity.getY() * Constants.MAX_SPEED,
                  omega * Constants.MAX_ANGULAR_SPEED);
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());
          drive.drive(speeds);
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * Constants.MAX_SPEED,
                      linearVelocity.getY() * Constants.MAX_SPEED,
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation());
              drive.drive(speeds);
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command joystickDriveHeading(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationXSupplier,
      DoubleSupplier rotationYSupplier) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // Obter a velocidade linear
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calcular o ângulo desejado a partir dos inputs do Right Axis
              double rotationX = MathUtil.applyDeadband(rotationXSupplier.getAsDouble(), DEADBAND);
              double rotationY = MathUtil.applyDeadband(rotationYSupplier.getAsDouble(), DEADBAND);
              Rotation2d targetRotation = new Rotation2d(Math.atan2(rotationY, rotationX));

              // Calcular a velocidade angular com um controlador PID
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), targetRotation.getRadians());

              // Converter para velocidades relativas ao campo e enviar comando
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * Constants.MAX_SPEED,
                      linearVelocity.getY() * Constants.MAX_SPEED,
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation());
              drive.drive(speeds);
            },
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command joystickDriveHeadingReef(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationXSupplier,
      DoubleSupplier rotationYSupplier) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // Obter a velocidade linear
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calcular o ângulo desejado a partir dos inputs do Right Axis
              double rotationX = MathUtil.applyDeadband(rotationXSupplier.getAsDouble(), DEADBAND);
              double rotationY = MathUtil.applyDeadband(rotationYSupplier.getAsDouble(), DEADBAND);
              Rotation2d currentRotation = new Rotation2d(Math.atan2(rotationY, rotationX));
              Rotation2d targetRotation = new Rotation2d();

              double angle = currentRotation.getDegrees();

              if (angle >= 150 || angle < -150) {
                targetRotation = Rotation2d.fromDegrees(180 + 180);
              } else if (angle >= -150 && angle < -90) {
                targetRotation = Rotation2d.fromDegrees(-120 + 180);
              } else if (angle >= -90 && angle < -30) {
                targetRotation = Rotation2d.fromDegrees(-60 + 180);
              } else if (angle >= -30 && angle < 30) {
                targetRotation = Rotation2d.fromDegrees(0 + 180);
              } else if (angle >= 30 && angle < 90) {
                targetRotation = Rotation2d.fromDegrees(60 + 180);
              } else if (angle >= 90 && angle < 150) {
                targetRotation = Rotation2d.fromDegrees(120 + 180);
              }

              // Calcular a velocidade angular com um controlador PID
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), targetRotation.getRadians());

              // Converter para velocidades relativas ao campo e enviar comando
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      -linearVelocity.getX() * Constants.MAX_SPEED,
                      -linearVelocity.getY() * Constants.MAX_SPEED,
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Blue;
              speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(-Math.PI))
                          : drive.getRotation());
              drive.drive(speeds);
            },
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command joystickDriveAimAtPoint(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double targetX,
      double targetY) {

    // Criação do controlador PID para controle de rotação
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construção do comando
    return Commands.run(
            () -> {
              // Obter velocidade linear a partir dos joysticks
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Pose atual do robô
              Pose2d currentPose = drive.getPose();

              // Calcula o ângulo desejado para o ponto (x, y)
              double desiredTheta =
                  Math.PI
                      + (Math.atan2(targetY - currentPose.getY(), targetX - currentPose.getX()));

              // Apply rotation deadband
              double omegaController =
                  MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omegaController = Math.copySign(omegaController * omegaController, omegaController);

              // Calcula a velocidade angular usando o controlador PID
              double omega =
                  angleController.calculate(drive.getRotation().getRadians(), desiredTheta);

              if (!(omegaController == 0)) {
                omega = omegaController * Constants.MAX_ANGULAR_SPEED;
              }

              // Converte as velocidades para referencia de campo e envia o comando
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * Constants.MAX_SPEED,
                      linearVelocity.getY() * Constants.MAX_SPEED,
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation());
              drive.drive(speeds);
            },
            drive)

        // Reseta o controlador PID quando o comando inicia
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command joystickDriveAimAtPoint(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Pose2d targetPose) {

    // Criação do controlador PID para controle de rotação
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construção do comando
    return Commands.run(
            () -> {
              // Obter velocidade linear a partir dos joysticks
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Pose atual do robô
              Pose2d currentPose = drive.getPose();

              // Calcula o ângulo desejado para o ponto (x, y)
              double desiredTheta =
                  Math.PI
                      + (Math.atan2(
                          targetPose.getY() - currentPose.getY(),
                          targetPose.getX() - currentPose.getX()));

              // Apply rotation deadband
              double omegaController =
                  MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omegaController = Math.copySign(omegaController * omegaController, omegaController);

              // Calcula a velocidade angular usando o controlador PID
              double omega =
                  angleController.calculate(drive.getRotation().getRadians(), desiredTheta);

              if (!(omegaController == 0)) {
                omega = omegaController * Constants.MAX_ANGULAR_SPEED;
              }

              // Converte as velocidades para referencia de campo e envia o comando
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * Constants.MAX_SPEED,
                      linearVelocity.getY() * Constants.MAX_SPEED,
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation());
              drive.drive(speeds);
            },
            drive)

        // Reseta o controlador PID quando o comando inicia
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command joystickDriveTowardsPoint(
    SwerveSubsystem drive,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier omegaSupplier,
    double targetX,
    double targetY) {

  final double baseTargetWeight = 0.3;
  final double maxEffectiveDistance = 2.0; // metros
  final double slowRadius = 0.4; // freio suave se estiver perto

  return Commands.run(
      () -> {
        double joystickX = xSupplier.getAsDouble();
        double joystickY = ySupplier.getAsDouble();

        // Deadband e processamento de rotação manual
        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
        omega = Math.copySign(omega * omega, omega);

        joystickX = MathUtil.applyDeadband(joystickX, DEADBAND);
        joystickY = MathUtil.applyDeadband(joystickY, DEADBAND);

        Pose2d currentPose = drive.getPose();
        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();

        double distanceToTarget = Math.hypot(dx, dy);

        // Vetor normalizado
        double tx = 0, ty = 0;
        if (distanceToTarget > 0.01) {
          tx = dx / distanceToTarget;
          ty = dy / distanceToTarget;
        }

        // Peso dinâmico
        double dynamicWeight = baseTargetWeight * Math.min(distanceToTarget / maxEffectiveDistance, 1.0);

        double blendedX = (1 - dynamicWeight) * joystickX + dynamicWeight * tx;
        double blendedY = (1 - dynamicWeight) * joystickY + dynamicWeight * ty;

        double blendedMagnitude = Math.hypot(blendedX, blendedY);
        if (blendedMagnitude > 1.0) {
          blendedX /= blendedMagnitude;
          blendedY /= blendedMagnitude;
        }

        // Freio vetorial se muito perto
        if (distanceToTarget < slowRadius) {
          double scale = distanceToTarget / slowRadius;
          blendedX *= scale;
          blendedY *= scale;
        }

        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        if (joystickX == 0.0 && joystickY == 0.0) {
          blendedX = 0;
          blendedY = 0;
        }

        ChassisSpeeds speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                blendedX * Constants.MAX_SPEED,
                blendedY * Constants.MAX_SPEED,
                omega * Constants.MAX_ANGULAR_SPEED,
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
        drive.drive(speeds);
      },
      drive);
}

  public static Command joystickDriveTowardsAimAtPoint(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double targetX,
      double targetY) {

    final double targetWeight = 0.2;

    final ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          double joystickX = -xSupplier.getAsDouble();
          double joystickY = -ySupplier.getAsDouble();

          joystickX = MathUtil.applyDeadband(joystickX, DEADBAND);
          joystickY = MathUtil.applyDeadband(joystickY, DEADBAND);

          Pose2d currentPose = drive.getPose();
          double currentX = currentPose.getX();
          double currentY = currentPose.getY();

          double targetVectorX = targetX - currentX;
          double targetVectorY = targetY - currentY;

          double targetMagnitude = Math.hypot(targetVectorX, targetVectorY);
          if (targetMagnitude > 0.01) {
            targetVectorX /= targetMagnitude;
            targetVectorY /= targetMagnitude;
          }

          double blendedX = (1 - targetWeight) * joystickX + targetWeight * targetVectorX;
          double blendedY = (1 - targetWeight) * joystickY + targetWeight * targetVectorY;

          double blendedMagnitude = Math.hypot(blendedX, blendedY);
          if (blendedMagnitude > 1.0) {
            blendedX /= blendedMagnitude;
            blendedY /= blendedMagnitude;
          }

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Blue;

          if (joystickX == 0.0 && joystickY == 0.0) {
            blendedX = 0;
            blendedY = 0;
          }

          double desiredTheta =
              Math.PI + (Math.atan2(targetY - currentPose.getY(), targetX - currentPose.getX()));

          double omegaController = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          omegaController = Math.copySign(omegaController * omegaController, omegaController);

          double omega = angleController.calculate(drive.getRotation().getRadians(), desiredTheta);

          if (!(omegaController == 0)) {
            omega = omegaController * Constants.MAX_ANGULAR_SPEED;
          }

          ChassisSpeeds speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  -blendedX * Constants.MAX_SPEED,
                  -blendedY * Constants.MAX_SPEED,
                  omega,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());
          drive.drive(speeds);
        },
        drive);
  }

  public static Command joystickDriveWithZoneRepulsion(
    SwerveSubsystem drive,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier omegaSupplier,
    Translation2d cornerA,
    Translation2d cornerB,
    double repulsionStrength) {

    return Commands.run(
      () -> {
        double joyX = xSupplier.getAsDouble();
        double joyY = ySupplier.getAsDouble();

        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
        omega = Math.copySign(omega * omega, omega);

        joyX = MathUtil.applyDeadband(joyX, DEADBAND);
        joyY = MathUtil.applyDeadband(joyY, DEADBAND);

        Pose2d pose = drive.getPose();
        double x = pose.getX();
        double y = pose.getY();

        double minX = Math.min(cornerA.getX(), cornerB.getX());
        double maxX = Math.max(cornerA.getX(), cornerB.getX());
        double minY = Math.min(cornerA.getY(), cornerB.getY());
        double maxY = Math.max(cornerA.getY(), cornerB.getY());

        Translation2d repulsion = new Translation2d();

        if (x > minX && x < maxX && y > minY && y < maxY) {
          // Calcular vetor de repulsão para fora do centro do retângulo
          Translation2d center = new Translation2d((minX + maxX) / 2.0, (minY + maxY) / 2.0);
          Translation2d delta = pose.getTranslation().minus(center);
          double norm = delta.getNorm();
          Translation2d directionOut = norm > 1e-6 ? delta.div(norm) : new Translation2d();
                    repulsion = directionOut.times(repulsionStrength);
        }

        double vx = joyX + repulsion.getX();
        double vy = joyY + repulsion.getY();

        double magnitude = Math.hypot(vx, vy);
        if (magnitude > 1.0) {
          vx /= magnitude;
          vy /= magnitude;
        }

        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        ChassisSpeeds speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx * Constants.MAX_SPEED,
                vy * Constants.MAX_SPEED,
                omega * Constants.MAX_ANGULAR_SPEED,
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
        drive.drive(speeds);
      },
      drive
    );
  }
}
