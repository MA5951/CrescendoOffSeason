
package frc.robot.Subsystem.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;


public class LED extends SubsystemBase {

  private static LED led;

  private DigitalOutput redOutput;
  private DigitalOutput greenOutput;
  private DigitalOutput blueOutput;

  public LED() {
    redOutput = new DigitalOutput(2);
    greenOutput = new DigitalOutput(3);
    blueOutput = new DigitalOutput(4);

    // redOutput.set(true);
    // greenOutput.set(false);
    // blueOutput.set(false);
  }



  public static LED getInstance() {
    if (led == null) {
      led = new LED();
    }
    return led;
  }

  @Override
  public void periodic() {

    // //Red
    // redOutput.set(true);
    // greenOutput.set(true);
    // blueOutput.set(true);

    // //Red
    // redOutput.set(true);
    // greenOutput.set(true);
    // blueOutput.set(false);

    
  }
}