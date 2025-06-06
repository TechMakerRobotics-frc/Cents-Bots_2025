package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.ZoneLocates.Zones;

public class ZoneTriggers {

  private final Drive drive;

  private final Trigger blueLeftStation;
  private final Trigger blueRightStation;
  private final Trigger redLeftStation;
  private final Trigger redRightStation;
  private final Trigger blueProcessor;
  private final Trigger redProcessor;
  private final Trigger blueReefSecurity;
  private final Trigger redReefSecurity;
  private final Trigger blueReefSubsystem;
  private final Trigger redReefSubsystem;
  private final Trigger notZone;

  public ZoneTriggers(Drive drive) {
    this.drive = drive;

    blueLeftStation = createTrigger(Zones.BLUE_LEFT_STATION);
    blueRightStation = createTrigger(Zones.BLUE_RIGHT_STATION);
    redLeftStation = createTrigger(Zones.RED_LEFT_STATION);
    redRightStation = createTrigger(Zones.RED_RIGHT_STATION);
    blueProcessor = createTrigger(Zones.BLUE_PROCESSOR);
    redProcessor = createTrigger(Zones.RED_PROCESSOR);
    blueReefSecurity = createTrigger(Zones.REEF_SECURITY_ZONE_BLUE);
    redReefSecurity = createTrigger(Zones.REEF_SECURITY_ZONE_RED);
    blueReefSubsystem = createTrigger(Zones.REEF_SUBSYSTEM_ZONE_BLUE);
    redReefSubsystem = createTrigger(Zones.REEF_SUBSYSTEM_ZONE_RED);
    notZone = createTrigger(Zones.NOT_ZONE);
  }

  private Trigger createTrigger(Zones zone) {
    return new Trigger(
        () -> {
          Zones[] zones = drive.getCurrentZones();
          if (zones.length == 0) return false;

          for (Zones currentZone : zones) {
            if (currentZone == zone) {
              return true;
            }
          }
          return false;
        });
  }

  public Trigger blueLeftStation() {
    return blueLeftStation;
  }

  public Trigger blueRightStation() {
    return blueRightStation;
  }

  public Trigger redLeftStation() {
    return redLeftStation;
  }

  public Trigger redRightStation() {
    return redRightStation;
  }

  public Trigger blueProcessor() {
    return blueProcessor;
  }

  public Trigger redProcessor() {
    return redProcessor;
  }

  public Trigger blueReefSecurity() {
    return blueReefSecurity;
  }

  public Trigger redReefSecurity() {
    return redReefSecurity;
  }

  public Trigger blueReefSubsystem() {
    return blueReefSubsystem;
  }

  public Trigger redReefSubsystem() {
    return redReefSubsystem;
  }

  public Trigger notZone() {
    return notZone;
  }
}
