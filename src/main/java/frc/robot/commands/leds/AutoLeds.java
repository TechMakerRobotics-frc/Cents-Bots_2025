package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.Led;

public class AutoLeds extends Command {
  private final Led led;
  private final Cradle cradle;
  private final CommandXboxController operator;
  private Timer timeout = new Timer();

  private enum States {
    CradleSensor,
    AprilTag,
    Idle
  }

  private States state;
  private boolean lastCradleSensor = false;

  public AutoLeds(
      Led led,
      Cradle cradle,
      // Vision vision,
      Arm arm,
      Elevator elevator,
      CommandXboxController driver,
      CommandXboxController operator) {
    this.led = led;
    this.cradle = cradle;
    this.operator = operator;
    addRequirements(led);
  }

  private void setAllianceColor() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      // led.setColor(Color.kBlue);
      led.setAllColorRGB(0, 0, 255);
    } else {
      led.setColor(Color.kRed);
    }
  }

  @Override
  public void initialize() {
    setAllianceColor();
    state = States.Idle;
  }

  @Override
  public void execute() {
    switch (state) {
      case Idle:
        if (cradle.frontSensorIsTrue() && !lastCradleSensor) {
          lastCradleSensor = true;
          led.setColor(Color.kGreen);
          timeout.restart();
          state = States.CradleSensor;
          operator.setRumble(RumbleType.kBothRumble, 1);
        }
        if (!cradle.frontSensorIsTrue()) {
          lastCradleSensor = false;
        }
        break;
      case CradleSensor:
        if (timeout.get() > 1) {
          setAllianceColor();
          operator.setRumble(RumbleType.kBothRumble, 0);
          state = States.Idle;
        }

        break;
      case AprilTag:
        break;
      default:
        break;
    }
  }
}
