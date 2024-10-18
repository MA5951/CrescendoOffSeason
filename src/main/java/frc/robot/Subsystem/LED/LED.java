
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
import frc.robot.RobotConstants;
import frc.robot.RobotControl.SuperStructure;


public class LED extends SubsystemBase {

  private static LED led;

  private DigitalOutput redOutput;
  private DigitalOutput greenOutput;
  private DigitalOutput blueOutput;
  public boolean isIntake;

  public LED() {
    redOutput = new DigitalOutput(2);
    greenOutput = new DigitalOutput(3);
    blueOutput = new DigitalOutput(4);

    Red();

    //All false is green like
    //redOutput.set(false);
    // greenOutput.set(false);
    // blueOutput.set(false);


  }

  public void Red() {
    //Red
    redOutput.set(true);
    greenOutput.set(true);
    blueOutput.set(false);
  }

  public void Green() {
    //Green
    redOutput.set(false);
    greenOutput.set(false);
    blueOutput.set(false);
  }

  public static LED getInstance() {
    if (led == null) {
      led = new LED();
    }
    return led;
  }

  @Override
  public void periodic() {

    if (!isIntake) {
      if (RobotConstants.SUPER_STRUCTURE.isNote() && RobotConstants.SUPER_STRUCTURE.getDistanceToTag() < RobotConstants.DISTANCE_TO_SHOOT) {
        Green();
      } else {
        Red();
      }
    }
  }
}