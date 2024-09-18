// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.LED;

import com.ma5951.utils.Leds.LEDBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.PortMap;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;

public class LED extends LEDBase {

    private static LED led;
    private Runnable emergencyEffect;
    private double emergencyEndTime;

    public LED() {
        super(PortMap.Leds.ledPort, LedConstants.ledLength);
    }

    public static LED getInstance() {
        if (led == null) {
            led = new LED();
        }
        return led;
    }

    public void setEmergencyEffect(Runnable effect, double duration) {
        emergencyEffect = effect;
        emergencyEndTime = Timer.getFPGATimestamp() + duration;
    }

    @Override
    public void periodic() {
        // Runnable effect = getCurrentEffect();
        // if (effect != null) {
        //     effect.run();
        // }
        // super.periodic(); 
        setSolidColor(LedConstants.MAcolor);
        updateLeds();
        //System.out.println("PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP");
    }

    private Runnable getCurrentEffect() {
        // if (emergencyEffect != null) {
        //     return () -> {
        //         emergencyEffect.run();
        //         if (Timer.getFPGATimestamp() >= emergencyEndTime) {
        //             emergencyEffect = null; // Reset emergency effect
        //         }
        //     };
        // }

        // if (DriverStation.isTeleopEnabled()) {
        //     //System.out.println("Teleop");
        //     if (Feeder.getInstance().getTargetState() == FeederConstants.NOTE_ADJUSTING) {
        //         return () -> blinkColorPattern(0.2, LedConstants.CONE_YELLOW, LedConstants.BLACK);
        //     } else if (RobotContainer.currentRobotState == RobotConstants.WARMING) {
        //         return () -> smoothWaveColorPattern(2, 1, 2, new Color[]{LedConstants.GREEN, LedConstants.RED}); 
        //     } else if (RobotContainer.currentRobotState == RobotConstants.INTAKE) {
        //         return () -> smoothWaveColorPattern(2, 1, 1, new Color[]{LedConstants.PURPLE, LedConstants.CYAN}); 
        //     } else if (RobotContainer.currentRobotState == RobotConstants.SOURCE_INTAKE) {
        //         return () -> smoothWaveColorPattern(2, 1, 1, new Color[]{LedConstants.BLUE, LedConstants.CYAN}); 
        //     } else if (RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING) {
        //         return () -> blinkColorPattern(0.2, LedConstants.WHITE, LedConstants.BLACK);
        //     } else if (RobotContainer.currentRobotState == RobotConstants.PRESET_SHOOTING) {
        //         return () -> blinkColorPattern(0.2, LedConstants.CYAN, LedConstants.RED);
        //     } else {
        //         return () -> smoothWaveColorPattern(2, 1, 1, new Color[]{LedConstants.GREEN, LedConstants.RED}); 
        //     }  
        // } else if (DriverStation.isAutonomousEnabled()) {
        //     //System.out.println("Auto");
        //     return () -> smoothWaveColorPattern(3, 1, 1, new Color[]{LedConstants.CONE_YELLOW, LedConstants.CUBE_PURPLE, LedConstants.CYAN});
        // } else if (DriverStation.isDisabled()) {
        //     if (DriverStation.isFMSAttached()) {
        //         if (DriverStation.getAlliance().get() == Alliance.Blue) {
        //             //System.out.println("Blue Alliance");
        //             return () -> smoothWaveColorPattern(2, 1, 0.2, new Color[]{LedConstants.BLACK, LedConstants.BLUE}); 
        //         } else if (DriverStation.getAlliance().get() == Alliance.Red) {
        //             //System.out.println("Red Alliance");
        //             return () -> smoothWaveColorPattern(2, 1, 0.2, new Color[]{LedConstants.BLACK, LedConstants.RED}); 
        //         } else {
        //             //System.out.println("None Alliance");
        //             return () -> smoothWaveColorPattern(2, 1, 0.2, new Color[]{LedConstants.BLACK, LedConstants.PURPLE}); 
        //         }
        //     } else {
        //         //System.out.println("Not Attached");
        //         return () -> blinkColorPattern(1, LedConstants.PURPLE, LedConstants.BLACK); 
        //     }
        // } else {
        //     //System.out.println("MAcolorrrrrrr");
        //     return () -> setSolidColor(LedConstants.MAcolor); 
        // }
        return () -> setSolidColor(LedConstants.MAcolor); 
    }
}