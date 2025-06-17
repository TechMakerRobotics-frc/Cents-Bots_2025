package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotState;
import frc.robot.commands.drive.GoToFieldPose;
// import frc.robot.commands.climber.Climb;
// import frc.robot.commands.climber.MinimalClimb;
// import frc.robot.commands.climber.ReverseClimb;
// import frc.robot.commands.climber.StopClimber;
// import frc.robot.commands.leds.*;
// import frc.robot.commands.reef.*;
// import frc.robot.commands.zero.Zeroelevator;
// import frc.robot.commands.zero.zeroArm;
// import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.arm.ArmIO;
// import frc.robot.subsystems.arm.ArmIOSpark;
// import frc.robot.subsystems.climber.Climber;
// import frc.robot.subsystems.climber.ClimberIO;
// import frc.robot.subsystems.climber.ClimberIOSpark;
// import frc.robot.subsystems.cradle.Cradle;
// import frc.robot.subsystems.cradle.CradleConstants;
// import frc.robot.subsystems.cradle.CradleIO;
// import frc.robot.subsystems.cradle.CradleIOSpark;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.elevator.ElevatorIO;
// import frc.robot.subsystems.elevator.ElevatorIOSpark;
// import frc.robot.subsystems.funnel.Funnel;
// import frc.robot.subsystems.funnel.FunnelIO;
// import frc.robot.subsystems.funnel.FunnelIOSpark;
// import frc.robot.subsystems.led.Led;
// import frc.robot.subsystems.led.LedIO;
// import frc.robot.subsystems.led.LedIOReal;
// import frc.robot.subsystems.led.LedIOSim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.GeomUtil;
import frc.robot.util.flippable.FlippablePose2d;

import java.io.File;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  // private final Led leds;
  // private final Elevator elevator;
  // private final Arm arm;
  // private final Cradle cradle;
  // private final Funnel funnel;
  // private final Climber climber;

  private SwerveDriveSimulation driveSimulation = null;

  // control leds
  // private int currentLedState = 0;
  // private final Command[] ledCommands;

  // State Machine
  private RobotState currentState = RobotState.NOT_ZONE;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  // private final CommandXboxController operatorController = new CommandXboxController(1);

  // private final Joystick tablet = new Joystick(1);
  // private final JoystickButton level1 = new JoystickButton(tablet, 13);
  // private final JoystickButton level2 = new JoystickButton(tablet, 14);
  // private final JoystickButton level3 = new JoystickButton(tablet, 15);
  // private final JoystickButton level4 = new JoystickButton(tablet, 16);
  // private final JoystickButton coralStationLeft = new JoystickButton(tablet, 17);
  // private final JoystickButton coralStationRight = new JoystickButton(tablet, 18);

  // The robot's subsystems and commands are defined here...

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  Command driveFieldOrientedAngularVelocity;
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driveController.getLeftY() * -1,
              () -> driveController.getLeftX() * -1)
          .withControllerRotationAxis(() -> driveController.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(driveController::getRightX, driveController::getRightY)
          .headingWhile(true);

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity
          .copy()
          .robotRelative(driveController.a().getAsBoolean())
          .allianceRelativeControl(false);

  // private Trigger climb;
  // private Trigger reverseClimb;
  Command driveFieldOrientedAnglularVelocity;
  Command driveRobotOrientedAnglularVelocity;

  private static final Integer kL1 = 1;
  private static final Integer kAutoLeave = 0;
  // private Integer m_autoSelected;
  private final SendableChooser<Integer> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    driveRobotOrientedAnglularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    drivebase.setDefaultCommand(driveRobotOrientedAnglularVelocity);
    m_chooser.setDefaultOption("Centro L1", kL1);

    m_chooser.addOption("Somente sair", kAutoLeave);
    SmartDashboard.putData("Autos", m_chooser);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        // leds = new Led(new LedIOReal());
        // elevator = new Elevator(new ElevatorIOSpark());
        // arm = new Arm(new ArmIOSpark());
        // cradle = new Cradle(new CradleIOSpark());
        // funnel = new Funnel(new FunnelIOSpark());
        // climber = new Climber(new ClimberIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        // leds = new Led(new LedIOSim());
        // elevator = new Elevator(new ElevatorIO() {});
        // arm = new Arm(new ArmIO() {});
        // cradle = new Cradle(new CradleIO() {});
        // funnel = new Funnel(new FunnelIO() {});
        // climber = new Climber(new ClimberIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations

        // leds = new Led(new LedIO() {});
        // elevator = new Elevator(new ElevatorIO() {});
        // arm = new Arm(new ArmIO() {});
        // cradle = new Cradle(new CradleIO() {});
        // funnel = new Funnel(new FunnelIO() {});
        // climber = new Climber(new ClimberIO() {});
        break;
    }

    // ledCommands =
    //     new Command[] {
    //       new LedRed(leds),
    //       new LedGreen(leds),
    //       new LedBlue(leds),
    //       new LedOff(leds),
    //       new LedWhite(leds),
    //       new LedYellow(leds),
    //       new LedCian(leds),
    //       new LedRainbow(leds)
    //     };

    // climb = new Trigger(() -> (driveController.getRightTriggerAxis() >= 0.4));
    // reverseClimb = new Trigger(() -> (driveController.getLeftTriggerAxis() >= 0.4));

    // leds.setDefaultCommand(
    //     new AutoLeds(leds, cradle, arm, elevator, driveController, operatorController));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveController
        .start()
        .onTrue(
            new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), new Rotation2d()))));


    driveController.y().onTrue(new PathPlannerAuto("Autotestal"));

    // // leds
    // operatorController
    //     .rightStick()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               currentLedState = (currentLedState + 1) % ledCommands.length;
    //               Command nextCommand = ledCommands[currentLedState];
    //               nextCommand.schedule();
    //             }));
    // operatorController.a().onTrue(new L1(arm, elevator, cradle));

    // operatorController.b().onTrue(new L2(arm, elevator, cradle));

    // operatorController.x().onTrue(new L3(arm, elevator, cradle));

    // operatorController.y().onTrue(new L4(arm, elevator, cradle));

    // operatorController.povDown().onTrue(new L0(arm, elevator, cradle));

    // operatorController.povLeft().onTrue(new DeliveryCoral(cradle));
    // operatorController.povRight().onTrue(new IntakeCoral(cradle, funnel));
    // operatorController
    //     .povUp()
    //     .onTrue(new InstantCommand(() -> cradle.setInverted(CradleConstants.INTAKE_ADJUST),
    // cradle))
    //     .onFalse(new InstantCommand(() -> cradle.set(0), cradle));

    // operatorController.leftBumper().whileTrue(new Zeroelevator(elevator));
    // operatorController.rightBumper().whileTrue(new zeroArm(arm));
    // operatorController.leftTrigger(0.5).onTrue(new
    // InstantCommand(()->cradle.set(CradleConstants.INTAKE_BACK),cradle))
    //                                     .onFalse(new InstantCommand(()->cradle.set(0),cradle));
    // climb.onTrue(new Climb(climber)).onFalse(new StopClimber(climber));

    // reverseClimb.onTrue(new ReverseClimb(climber)).onFalse(new StopClimber(climber));

    // driveController.leftStick().onTrue(new MinimalClimb(climber).ignoringDisable(true));

    /*level1.onTrue(new L1(arm, elevator, cradle));
    level2.onTrue(new L2(arm, elevator, cradle));
    level3.onTrue(new L3(arm, elevator, cradle));
    level4.onTrue(new L4(arm, elevator, cradle));
    coralStationLeft.onTrue(new L0(arm, elevator, cradle));
    coralStationRight.onTrue(new L0(arm, elevator, cradle));*/

    // driveController.leftBumper().whileTrue(new AlignLeftReef(drivebase, vision));
    // driveController.rightBumper().whileTrue(new AlignRightReef(drivebase, vision));
    // driveController
    //     .povUp()
    //     .onTrue(new InstantCommand(() -> arm.runPosition(ArmConstants.ARM_MAX_POSITION), arm));
    // driveController
    //     .povDown()
    //     .onTrue(new InstantCommand(() -> arm.runPosition(ArmConstants.ARM_MIN_POSITION), arm));
    // driveController
    //     .povRight()
    //     .onTrue(new InstantCommand(() -> cradle.set(0.1), cradle))
    //     .onFalse(new InstantCommand(() -> cradle.set(0.0), cradle));
    // driveController.povLeft().onTrue(new InstantCommand(() -> cradle.resetEncoder(), cradle));

    // driveController.y().onTrue(new LedSolidColor(leds, Color.kBrown));

    // Triggers do sistema
    // startAuto.onTrue(new InstantCommand(()->cradle.setZero(),cradle));
    // startTeleop.onTrue(new InstantCommand(() -> cradle.setZero(), cradle));
    // limitSpeed.onTrue(new InstantCommand(()->drivebase.setLimitSpeed(true)))
    //          .onFalse(new InstantCommand(()->drivebase.setLimitSpeed(true)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // switch (m_chooser.getSelected()) {
    //   case 1:

    //     new AutoA1(drivebase, arm, elevator, cradle);
    //   default:
    //     return new DriveToX(drivebase, 1);
    // }
    return Commands.none();
  }

  @AutoLogOutput(key = "StateMachine/CurrentState")
  public RobotState getCurrentState() {
    return currentState;
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
