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
        Runnable effect = getCurrentEffect();
        if (effect != null) {
            effect.run();
        }
        super.periodic(); 
    }

    private Runnable getCurrentEffect() {
        if (emergencyEffect != null) {
            return () -> {
                emergencyEffect.run();
                if (Timer.getFPGATimestamp() >= emergencyEndTime) {
                    emergencyEffect = null; // Reset emergency effect
                }
            };
        }

        if (DriverStation.isEnabled()) {
            return this::rainbowColorPatternDrivers; 
        } else if (DriverStation.isAutonomous()) {
            return () -> smoothWaveColorPattern(3, 1, 1, new Color[]{LedConstants.CONE_YELLOW, LedConstants.CUBE_PURPLE, LedConstants.CYAN});
        } else if (DriverStation.isDisabled()) {
            if (DriverStation.isFMSAttached()) {
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return () -> smoothWaveColorPattern(2, 1, 0.2, new Color[]{LedConstants.BLACK, LedConstants.BLUE}); 
                } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                    return () -> smoothWaveColorPattern(2, 1, 0.2, new Color[]{LedConstants.BLACK, LedConstants.RED}); 
                } else {
                    return () -> smoothWaveColorPattern(2, 1, 0.2, new Color[]{LedConstants.BLACK, LedConstants.PURPLE}); 
                }
            } else {
                return () -> blinkColorPattern(1, LedConstants.PURPLE, LedConstants.BLACK); 
            }
        } else {
            return () -> setSolidColor(LedConstants.MAcolor); 
        }
    }
}