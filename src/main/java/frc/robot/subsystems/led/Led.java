package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private final LedIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  public Led(LedIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Led", inputs);
  }

  public void setAllColorRGB(int red, int green, int blue) {
    io.setAllColorRGB(red, green, blue);
  }

  public void setAllColorHSV(int hue, int saturation, int value) {
    io.setAllColorHSV(hue, saturation, value);
  }

  public void setHSV(int index, int h, int s, int v) {
    io.setHSV(index, h, s, v);
  }

  public void setColor(Color color) {
    setAllColorRGB((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  public void clear() {
    io.clear();
  }
}
