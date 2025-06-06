package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.Led;

public class LedSolidColor extends InstantCommand {

  public LedSolidColor(Led leds, Color color) {
    super(
        () ->
            leds.setAllColorRGB(
                (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255)),
        leds);
  }
}
