
package frc.robot.Subsystem.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;


public class LED extends SubsystemBase {

  private static LED led;

  private AddressableLED leds;
  private AddressableLEDBuffer ledBuffer;
  private int firstHue = 0;
  private double lastChange;
  private boolean on;
  private Color currentColor;

  public LED() {
    leds = new AddressableLED(PortMap.Leds.ledPort);
    ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
    leds.setLength(ledBuffer.getLength());
    leds.setData(ledBuffer);
    leds.start();
    currentColor = LedConstants.BLUE;

  }

  public void setSolidColor(Color color) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }

    leds.setData(ledBuffer);
  }

  public void smoothWaveColorPattern(int numColors, double period, double speed, Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp();

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      double position = ((double) i / ledBuffer.getLength()) + (elapsedTime * speed / period);
      double progress = position - (int) position;

      int startColorIndex = (int) (position % numColors);
      int endColorIndex = (startColorIndex + 1) % numColors;
      Color startColor = colors[startColorIndex];
      Color endColor = colors[endColorIndex];

      Color currentColor = new Color(
              startColor.red + (endColor.red - startColor.red) * progress,
              startColor.green + (endColor.green - startColor.green) * progress,
              startColor.blue + (endColor.blue - startColor.blue) * progress
      );

      ledBuffer.setLED(i, currentColor);
    }

    leds.setData(ledBuffer);
  }

  public void blinkColorPattern( double interval,Color colorOne, Color colorTwo) {
    double timestamp = Timer.getFPGATimestamp();
    if (timestamp - lastChange > interval) {
        on = !on;
        lastChange = timestamp;
    }
    if (on) {
        setSolidColor(colorOne);
    }
    else {
        setSolidColor(colorTwo);
    }

    leds.setData(ledBuffer);
  }

  public void updateLeds() {
    leds.setData(ledBuffer);
  }

  public static LED getInstance() {
    if (led == null) {
      led = new LED();
    }
    return led;
  }

  @Override
  public void periodic() {

    //blinkColorPattern( 1, LedConstants.CONE_YELLOW, LedConstants.BLUE);
    setSolidColor(LedConstants.GREEN);
    updateLeds();
  }
}